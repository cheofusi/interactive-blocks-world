module RobotArm
    (
        updateArm,
        armIsIdle
    )
where

import qualified Data.Map as Map
import qualified Data.Set as Set
import qualified Data.Sequence as Seq
import Control.Lens.Operators ( (^.), (.~), (&), (?~), (%~) )
import Control.Lens.Tuple ( Field1(_1), Field2(_2) )
import Control.Concurrent.Async

import Graphics.Gloss

import World
import RobotBrain
import Utils



-- This module simulates arm and block displacement by generating a sequence of maneuvers for each
-- task it is given. A maneuver involves either moving to a position, grasping a block, or ungrasping
-- a block.

-- PUT_ON b p1 p2  => The robot has to move the block b from the position p1 to the position p2. The
--                    corresponding maneuver sequence would be;
--                  (i)   Moving to p1
--                  (ii)  Grasping b at p1 
--                  (iii) Moving to p2
--                  (iv)  Ungrasping b at p2
-- For the arm to move from a position p1 to a position p2, it
--                  (i)   computes the height h of the highest block stack between p1 & p2.
--                  (ii)  moves vertically upwards until its height above the table is exactly h
--                  (iii) moves horizontally until it is directly above p2.
--                  (iv)  moves vertically downwards until it's at p2.

-- READY_ARM_ON b  => The robot positions itself right on the block b.


updateArm :: Float -> World -> IO World
updateArm dt world =
    if null (rArm ^. armManeuverSeq) then
        if null (rArm ^. armTaskSeq) then 
            do
                putStrLn "Done"
                asyncInput <- async readCmdLineInput
                return $ world 
                    & arm . isBusy .~ False
                    & userInputBuffer .~ asyncInput
        else
            let
                armManeuverSeq' = calcManeuverSeq (Seq.index (rArm ^. armTaskSeq) 0) world
                armTaskSeq' = Seq.drop 1 (rArm ^. armTaskSeq)
            in
                return $ world
                & arm . armTaskSeq .~ armTaskSeq'
                & arm . armManeuverSeq .~ armManeuverSeq'
    else
        case Seq.index (rArm ^. armManeuverSeq) 0 of
            MoveToPosition pos -> return $ world & arm .~ moveArm dt pos rArm

            Grasp b ->
                let
                    blockObj = blockObject b blocks
                in
                    return $ world
                    & arm .~ graspBlock blockObj rArm
                    & blockSet . _1 .~ Map.delete (blockObj ^. blockId) blocks

            UnGraspOn b ->
                let
                    (arm', ungraspedBlock) = unGraspBlock rArm
                    ungraspedBlockId = ungraspedBlock ^. blockId
                in
                    return $ world
                    & arm .~ arm'
                    & blockSet . _1 .~ Map.insert (ungraspedBlock ^. blockId) ungraspedBlock blocks
                    & blockSet . _2 %~ Set.map (\(b1, b2) -> if b1 == ungraspedBlockId then (b1, b) else (b1, b2))
    where
        blocks = world ^. (blockSet . _1)
        rArm = world ^. arm


calcManeuverSeq :: Task -> World -> ManeuverSeq
calcManeuverSeq task world =
    case task of
        PUT_ON b2 b1 emptyPosOnb2 -> 
            let
                startPos = (blockPosX b1 blocks, blockPosH b1 blocks + blockLength b1 blocks)

                -- blockset' is defined to make sure 2nd call to heightOfTallestBlockStackInRange
                -- understands that b1 has been grabbed and doesn't include it in its calculations
                blockset' = blockset & _2 %~ Set.map (\(b1', b2') -> if b1' == b1 then (b1', "") else (b1', b2'))

                h' = lowestEmptyHeightLevelInxRange (fst rArmPos, fst startPos) blockset + 10
                h'' = lowestEmptyHeightLevelInxRange (fst startPos, fst emptyPosOnb2) blockset' + 10
            in
                (enumArmMoveSeq rArmPos startPos h' Seq.|> Grasp b1)
                Seq.>< 
                (enumArmMoveSeq startPos emptyPosOnb2 h'' Seq.|> UnGraspOn b2)

        READY_ARM_ON b ->
            let
                startPos = (blockPosX b blocks, blockPosH b blocks + blockLength b blocks)
                h' = lowestEmptyHeightLevelInxRange (fst rArmPos, fst startPos) blockset + 10
            in
                enumArmMoveSeq rArmPos startPos h'
    where
        rArmPos = world ^. (arm . armPos)
        blockset = world ^. blockSet
        blocks = fst blockset


enumArmMoveSeq :: Position -> Position -> Float -> ManeuverSeq
enumArmMoveSeq (x1, _) (x2, h2) h = Seq.fromList $
    if x1 /= x2 then 
        [ MoveToPosition (x1, h)
        , MoveToPosition (x2, h)
        , MoveToPosition (x2, h2) ]

    else [ MoveToPosition (x2, h2) ]


moveArm :: Float -> Position -> Arm -> Arm
moveArm dt (target_x, target_h) rArm =
    let
        (x, h) = rArm ^. armPos
        (dx, dh) = (target_x - x, target_h - h)
        closeEnough = (abs dx < 0.25) && (abs dh < 0.25)
        x' = if closeEnough then target_x else x + dx * dt * armMoveSpeed
        h' = if closeEnough then target_h else h + dh * dt * armMoveSpeed
        armManeuverSeq' =
            if closeEnough then Seq.drop 1 (rArm ^. armManeuverSeq)
            else rArm ^. armManeuverSeq
    in
        rArm
        & armPos .~ (x', h')
        & armManeuverSeq .~ armManeuverSeq'


graspBlock :: Block -> Arm -> Arm
graspBlock blockObj rArm =
    let
        h' = armPosH rArm - (blockObj ^. (blockSize . _2))
        armManeuverSeq' = Seq.drop 1 (rArm ^. armManeuverSeq)
    in
        rArm
        & armPos . _2 .~ h'
        & graspedBlockObj ?~ blockObj
        & armManeuverSeq .~ armManeuverSeq'


unGraspBlock :: Arm -> (Arm, Block)
unGraspBlock rArm =
    let
        Just gblockObj = rArm ^. graspedBlockObj
        h' = armPosH rArm + (gblockObj ^. (blockSize . _2))
        armManeuverSeq' = Seq.drop 1 (rArm ^. armManeuverSeq)
        block' = gblockObj & blockPos .~ (armPosX rArm, armPosH rArm)

    in
        (,)
            (rArm
                & armPos . _2 .~ h'
                & graspedBlockObj .~ Nothing
                & armManeuverSeq .~ armManeuverSeq'
            )
            block'


armIsIdle :: Arm -> Bool
armIsIdle rArm = not (rArm ^. isBusy)


armMoveSpeed :: Float
armMoveSpeed = 5.0
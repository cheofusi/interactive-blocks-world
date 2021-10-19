{-# OPTIONS_GHC -Wno-incomplete-patterns #-}
module RobotBrain where

import qualified Data.Map as Map
import qualified Data.Set as Set
import qualified Data.Sequence as Seq
import Control.Lens.Operators ( (^.), (.~), (&) )
import Control.Lens.Tuple ( Field1(_1), Field2(_2) )

import World
import Utils


-- This module tries to break down a user command into a sequence of tasks to be executed
-- by the robot arm. It either fails with an error message and the task sequence it got so
-- far, or successfully returns a task sequence that satisfies the user command.


--type SimulatedBlockset = BlockSet
type Result = Either (String, TaskSeq) (BlockSet, TaskSeq)

data EmptyPosition = OnBlock BlockId Position | NoEmptyPosition


solveForInput :: UserCmd -> BlockSet -> Result
solveForInput uCmd blockset =
    case uCmd of
        PUTON b2 b1 -> putOn b2 b1 (blockset, Seq.Empty)
        PUTON_TABLE b -> putOnTable b (blockset, Seq.Empty)


putOn :: BlockId -> BlockId -> (BlockSet, TaskSeq) -> Result
putOn b2 b1 blockSetAndTaskSeq = do
    -- first check if b1 is already on b2
    b1_is_on_b2
    -- then check if b2's width >= b1's width
    b1_allowed_on_b2

    -- blocks removed from the top of b2 should not be placed on the block stack containing b1
    let w = blockWidth b1 blocks
    blockSetAndTaskSeq' <- createSpaceOn b2 w [b1] blockSetAndTaskSeq
    -- blocks removed from the top of b1 should not be placed on the block stack containing b2
    blockSetAndTaskSeq'' <- clearTopOf b1 [b2] blockSetAndTaskSeq'

    case findEmptyPositionOn b2 w (fst blockSetAndTaskSeq'') of
        -- This shouldn't match Nothing since the bind function for the Either monad would
        -- have returned after the call to createSpaceOn
        Nothing -> Left ("Fatal Error! This should not happen!!", snd blockSetAndTaskSeq'')
        Just emptyPosOnb2 ->
            Right $ addPutOnTask b2 b1 emptyPosOnb2 blockSetAndTaskSeq''

    where
        ((blocks, blockPlacements), taskSeq) = blockSetAndTaskSeq
        b1_is_on_b2 =
            case blocksOn b2 blockPlacements of
                [] -> Right ()
                blocksOnb2 ->
                    if b1 `elem` blocksOnb2 then Left (b1 ++ " is already on " ++ b2, taskSeq)
                    else Right ()
        b1_allowed_on_b2 =
            if blockWidth b2 blocks >= blockWidth b1 blocks then
                if createsInvalidBlockStack b2 b1 blocks then
                    Left ("Placing " ++ b1 ++ " on " ++ b2 
                        ++ " creates a block stack that's too high", taskSeq)
                else Right ()
            else Left (b1 ++ " cannot be placed on " ++ b2, taskSeq)


-- | returns Left errStr if no space is found on the table
putOnTable :: BlockId -> (BlockSet, TaskSeq) -> Result
putOnTable b blockSetAndTaskSeq = do
    -- first check if b1 is already on b2
    b_is_on_table
    
    blockSetAndTaskSeq' <- clearTopOf b [] blockSetAndTaskSeq

    case findEmptyPositionOnTableFor b (fst blockSetAndTaskSeq') of
        NoEmptyPosition -> 
            Left 
                ("No empty position on the table was found for placing " ++ b,
                    snd blockSetAndTaskSeq'
                )
        OnBlock b' emptyPosOnb' -> do --b' here is B0
            Right $ addPutOnTask b' b emptyPosOnb' blockSetAndTaskSeq'
    where
        ((blocks, blockPlacements), taskSeq) = blockSetAndTaskSeq
        b_is_on_table = 
            if b `elem` baseBlocks blockPlacements then Left (b ++ " is already on the table", taskSeq)
            else Right ()


-- | Creates an empty position on block b that can hold a block of width w. Only creates the space and 
--   doesn't return the position. Assumes that (blockWidth b) >= w. So the caller should make sure of this
createSpaceOn :: BlockId -> Float -> [BlockId] -> (BlockSet, TaskSeq) -> Result
createSpaceOn b w blocksToNotPlaceAbove blockSetAndTaskSeq =
    case findEmptyPositionOn b w (fst blockSetAndTaskSeq) of
        Nothing -> do
            -- if (blockWidth b) >= w and no empty position is available, it means b has at least
            -- one block directly on it. find the widestBlockOnB, getRidOf it and recurse
            let blocksOnb = blocksOn b blockPlacements
            let foldFunc = \b1 b2 ->
                    if blockWidth b2 blocks > blockWidth b1 blocks
                        then b2
                    else b1
            let widestBlockOnB = foldl1 foldFunc blocksOnb
            blockSetAndTaskSeq' <- getRidOf widestBlockOnB (b:blocksToNotPlaceAbove) blockSetAndTaskSeq
            createSpaceOn b w blocksToNotPlaceAbove blockSetAndTaskSeq'
        Just _ ->
            -- There is a position on block b that can fit a block of width w. So simply return
            Right blockSetAndTaskSeq
    where
        ((blocks, blockPlacements), _) = blockSetAndTaskSeq


-- | Frees the top of block b
clearTopOf :: BlockId -> [BlockId] -> (BlockSet, TaskSeq) -> Result
clearTopOf b blocksToNotPlaceAbove blockSetAndTaskSeq =
    case blocksOn b blockPlacements of
        [] -> Right blockSetAndTaskSeq
        blocksOnb -> do
            let aBlockOnb = head blocksOnb
            blockSetAndTaskSeq' <- getRidOf aBlockOnb (b:blocksToNotPlaceAbove) blockSetAndTaskSeq
            clearTopOf b blocksToNotPlaceAbove blockSetAndTaskSeq'
    where
        ((_, blockPlacements), _) = blockSetAndTaskSeq


-- | Removes block b from its current block stack and places it at an empty position somewhere else,
--   where that empty position isn't in/on the block stack containing blockToNotPlaceOn
getRidOf :: BlockId -> [BlockId] -> (BlockSet, TaskSeq) -> Result
getRidOf b blocksToNotPlaceAbove blockSetAndTaskSeq = do
    blockSetAndTaskSeq' <- clearTopOf b blocksToNotPlaceAbove blockSetAndTaskSeq
    let emptyPos = findEmptyPositionFor b blocksToNotPlaceAbove (fst blockSetAndTaskSeq')
    case emptyPos of
        NoEmptyPosition -> Left ("Failed to find an empty position for placing " ++ b
                                , snd blockSetAndTaskSeq')
        OnBlock b' emptyPosOnb' ->
            Right $ addPutOnTask b' b emptyPosOnb' blockSetAndTaskSeq'


-- | Adds a new PUT_ON task to the current task sequence and updates the simulated blockset
--   accordingly.
addPutOnTask :: BlockId -> BlockId -> Position -> (BlockSet, TaskSeq)  -> (BlockSet, TaskSeq)
addPutOnTask b2 b1 emptyPosOnb2 ((blocks, blockPlacements), taskSeq) =
    let
        blockb1 = blockObject b1 blocks
        taskSeq' = taskSeq Seq.|> PUT_ON b2 b1 emptyPosOnb2
        blocks' = Map.insert b1 (blockb1 & blockPos .~ emptyPosOnb2) blocks
        blockPlacements' = Set.map (\(b1', b2') -> if b1' == b1 then (b1', b2) else (b1', b2'))
                            blockPlacements
    in
        (,) (blocks', blockPlacements') taskSeq'


-- | Finds an empty position for block b'
findEmptyPositionFor :: BlockId -> [BlockId] -> BlockSet -> EmptyPosition
findEmptyPositionFor b' blocksToNotPlaceAbove blockset =
    betterEmptyPosition
        (blockPosition b' (fst blockset))
        (findEmptyPositionOnTableFor b' blockset)
        (findEmptyPositionOnaBlockFor b' blocksToNotPlaceAbove blockset)
    

-- | Finds an empty position on the table for block b'
findEmptyPositionOnTableFor :: BlockId -> BlockSet -> EmptyPosition
findEmptyPositionOnTableFor b' blockset = 
    let 
        w = blockWidth b' blocks
        b_blockStackBase = blockStackBase b' blockPlacements
        b_blockStackBase_x1 = blockPosX1 b_blockStackBase blocks
        b_blockStackBase_x2 = blockPosX2 b_blockStackBase blocks
    in
        betterEmptyPosition
            (blockPosition b' blocks)
            (findEmptyPositionOnTableLeftOf b_blockStackBase_x1 w blockset)
            (findEmptyPositionOnTableRightOf b_blockStackBase_x2 w blockset)
    where
        (blocks, blockPlacements) = blockset


-- | Finds an empty position on the table to the left of x1 that can fit a block of width w
findEmptyPositionOnTableLeftOf :: Float -> Float -> BlockSet -> EmptyPosition
findEmptyPositionOnTableLeftOf x1 w blockset = 
    if x1 - w < (-300) then NoEmptyPosition
    else
        case closestBaseBlockToxFromLeft x1 blockset of
            Nothing -> OnBlock "B0" (x1 - w/2, 0)
            (Just closestBaseBlockTox1FromLeft) ->
                if blockIntersectsxRange 
                        closestBaseBlockTox1FromLeft 
                        (x1 - w, x1) 
                        (fst blockset) 
                    then
                    findEmptyPositionOnTableLeftOf 
                        (blockPosX1 closestBaseBlockTox1FromLeft (fst blockset))
                        w
                        blockset
                else
                    OnBlock "B0" (x1 - w/2, 0)


-- | Finds an empty position on the table to the right of x2 that can fit a block of width w
findEmptyPositionOnTableRightOf :: Float -> Float -> BlockSet -> EmptyPosition
findEmptyPositionOnTableRightOf x2 w blockset = 
    if x2 + w > 300 then NoEmptyPosition
    else
        case closestBaseBlockToxFromRight x2 blockset of
            Nothing -> OnBlock "B0" (x2 + w/2, 0)
            (Just closestBaseBlockTox2FromRight) ->
                if blockIntersectsxRange 
                        closestBaseBlockTox2FromRight 
                        (x2, x2 + w) 
                        (fst blockset) 
                    then 
                    findEmptyPositionOnTableRightOf 
                        (blockPosX2 closestBaseBlockTox2FromRight (fst blockset)) 
                        w 
                        blockset
                else
                    OnBlock "B0" (x2 + w/2, 0)


-- | Finds an empty position on a block for block b'
findEmptyPositionOnaBlockFor :: BlockId -> [BlockId] -> BlockSet -> EmptyPosition
findEmptyPositionOnaBlockFor b' blocksToNotPlaceAbove blockset =
    let 
        b_blockStackBase = blockStackBase b' blockPlacements 
        b_blockStackBase_x1 = blockPosX1 b_blockStackBase blocks
        b_blockStackBase_x2 = blockPosX2 b_blockStackBase blocks
    in
        case findEmptyPositionAbove b_blockStackBase b' blocksToNotPlaceAbove blockset of
            NoEmptyPosition -> 
                betterEmptyPosition
                    (blockPosition b' blocks)
                    (findEmptyPositionOnaBlockLeftOf b_blockStackBase_x1 b' blocksToNotPlaceAbove blockset)
                    (findEmptyPositionOnaBlockRightOf b_blockStackBase_x2 b' blocksToNotPlaceAbove blockset)
            
            emptyPos -> emptyPos
    where 
        (blocks, blockPlacements) = blockset


-- | Finds an empty position on a block for block b' that is to the left of x1
findEmptyPositionOnaBlockLeftOf :: Float -> BlockId -> [BlockId] -> BlockSet -> EmptyPosition
findEmptyPositionOnaBlockLeftOf x1 b' blocksToNotPlaceAbove blockset =
    case closestBaseBlockTox1FromLeft of
        Nothing -> NoEmptyPosition
        (Just closestBaseBlockFromLeft) ->
            case findEmptyPositionAbove closestBaseBlockFromLeft b' blocksToNotPlaceAbove blockset of
                NoEmptyPosition -> 
                    findEmptyPositionOnaBlockLeftOf
                        (blockPosX1 closestBaseBlockFromLeft (fst blockset))
                        b'
                        blocksToNotPlaceAbove
                        blockset
                
                emptyPos -> emptyPos
    where
        closestBaseBlockTox1FromLeft = closestBaseBlockToxFromLeft x1 blockset
    

-- | Finds an empty position on a block for block b' that is to the right of x2
findEmptyPositionOnaBlockRightOf :: Float -> BlockId -> [BlockId] -> BlockSet -> EmptyPosition
findEmptyPositionOnaBlockRightOf x2 b' blocksToNotPlaceAbove blockset =
    case closestBaseBlockTox2FromRight of
        Nothing -> NoEmptyPosition
        (Just closestBaseBlockFromRight) ->
            case findEmptyPositionAbove closestBaseBlockFromRight b' blocksToNotPlaceAbove blockset of
                NoEmptyPosition ->
                    findEmptyPositionOnaBlockRightOf
                        (blockPosX2 closestBaseBlockFromRight (fst blockset))
                        b'
                        blocksToNotPlaceAbove
                        blockset
                
                emptyPos -> emptyPos
    where
        closestBaseBlockTox2FromRight = closestBaseBlockToxFromRight x2 blockset


-- | Finds an empty position for block b' that is either directly on block b or on another block that
--   is above b (ala recursive defn) and in the same block stack as b.
findEmptyPositionAbove:: BlockId -> BlockId -> [BlockId] -> BlockSet -> EmptyPosition
findEmptyPositionAbove b b' blocksToNotPlaceAbove blockset =
    let
        (blocks, blockPlacements) = blockset
        w' = blockWidth b' blocks
        l' = blockLength b' blocks
        blockbIsValid =
            not (or [blockIsAbove aBlockToNotPlaceAbove b blocks | aBlockToNotPlaceAbove <- blocksToNotPlaceAbove])
            &&
            notElem b blocksToNotPlaceAbove 
            &&
            (blockWidth b blocks >= w') 
            &&
            not (createsInvalidBlockStack b b' blocks)
    in
        if blockbIsValid then
            case findEmptyPositionOn b w' blockset of
                Nothing ->
                    let
                        foldFunc = betterEmptyPosition (blockPosition b' blocks)
                        blocksOnb = blocksOn b blockPlacements
                        emptyPositionsAboveb =
                            [findEmptyPositionAbove aBlockOnb b' blocksToNotPlaceAbove blockset | aBlockOnb <- blocksOnb]
                    in
                        foldl foldFunc NoEmptyPosition emptyPositionsAboveb
                Just emptyPos ->
                    OnBlock b emptyPos
        else
            NoEmptyPosition


-- | Finds an empty position directly on block b that can fit a block of width w
--   Assumes that (blockWidth b) >= w. So the caller should make sure of this
findEmptyPositionOn :: BlockId -> Float -> BlockSet -> Maybe Position
findEmptyPositionOn b w (blocks, blockPlacements) =
    case blocksOn b blockPlacements of
        [] -> Just (blockb_x1 + w/2, h)
        blocksOnb ->
            -- check if the sum of the widths of the blocksOnb is at most (blockWidth b - w). If it
            -- surpasses then there isn't any space for a block of width w. If it doesn't then look 
            -- for the leftmost empty position on b that fits w. If one isn't found it means the blocksOnb
            -- have to be altered to fit w, and that shouldn't be done, so simply return Nothing
            let
                totalWidthOfBlocksOnb = sum $ fmap (`blockWidth` blocks) blocksOnb
            in
                if totalWidthOfBlocksOnb > (blockWidth b blocks - w) then Nothing
                else
                    let
                        findPositionThatFitsw x =
                            if x + w <= blockb_x2 then
                                let
                                    blocksOnbInxRange = blocksOnbIntersectingxRange x (x+w)
                                    x' = maximum $ fmap (`blockPosX2` blocks) blocksOnbInxRange
                                in
                                    if null blocksOnbInxRange then Just (x + w/2, h)
                                    else
                                        findPositionThatFitsw x'
                            else Nothing
                    in
                        findPositionThatFitsw blockb_x1
                    where
                        blocksOnbIntersectingxRange x1 x2 =
                            [aBlockOnb | aBlockOnb <- blocksOnb, blockIntersectsxRange aBlockOnb (x1, x2) blocks]
    where
        blockb_x1 = blockPosX1 b blocks
        blockb_x2 = blockPosX2 b blocks
        h = blockPosH b blocks + blockLength b blocks


-- | Returns the better of 2 empty positions that will serve as the destination for
--   placing a block that is currently at the position p.
betterEmptyPosition :: Position -> EmptyPosition -> EmptyPosition -> EmptyPosition
betterEmptyPosition p emptyPos1 emptyPos2 =
    case (emptyPos1, emptyPos2) of
        (NoEmptyPosition, NoEmptyPosition) -> NoEmptyPosition
        (_ , NoEmptyPosition) -> emptyPos1
        (NoEmptyPosition, _) -> emptyPos2
        (OnBlock _ pos1, OnBlock _ pos2) ->
            if manhattanDistance p pos1 <= manhattanDistance p pos2 then emptyPos1
            else emptyPos2


-- | Returns the closest base block to the left of x
closestBaseBlockToxFromLeft :: Float -> BlockSet -> Maybe BlockId
closestBaseBlockToxFromLeft x (blocks, blockPlacements) = foldl foldFunc Nothing (baseBlocks blockPlacements)
    where
        foldFunc :: Maybe BlockId -> BlockId -> Maybe BlockId
        foldFunc Nothing b = if blockPosX2 b blocks <= x then Just b else Nothing
        foldFunc (Just closestBaseBlockSoFar) b =
            if blockb_x2 > closestBaseBlockSoFar_x2 && blockb_x2 <= x then Just b
            else Just closestBaseBlockSoFar
            where
                blockb_x2 = blockPosX2 b blocks
                closestBaseBlockSoFar_x2 = blockPosX2 closestBaseBlockSoFar blocks


-- | Returns the closest base block to the right of x
closestBaseBlockToxFromRight :: Float -> BlockSet -> Maybe BlockId
closestBaseBlockToxFromRight x (blocks, blockPlacements) = foldl foldFunc Nothing (baseBlocks blockPlacements)
    where
        foldFunc :: Maybe BlockId -> BlockId -> Maybe BlockId
        foldFunc Nothing b = if blockPosX1 b blocks >= x then Just b else Nothing
        foldFunc (Just closestBaseBlockSoFar) b =
            if blockb_x1 < closestBaseBlockSoFar_x1 && blockb_x1 >= x then Just b
            else Just closestBaseBlockSoFar
            where
                blockb_x1 = blockPosX1 b blocks
                closestBaseBlockSoFar_x1 = blockPosX1 closestBaseBlockSoFar blocks


-- | Returns the lowest height level in the range [min x1 x2, max x1 x2] that is free of any
--   blocks i.e the (blockPosH + blockLength) of the heighest block in the said xrange.
lowestEmptyHeightLevelInxRange :: (Float, Float) -> BlockSet -> Float
lowestEmptyHeightLevelInxRange (x1, x2) (blocks, blockPlacements) = 
    maximum (fmap lowestEmptyHeightLevelInxRange' (baseBlocks blockPlacements))
    where
        lowestEmptyHeightLevelInxRange' b = 
            if blockIntersectsxRange b (x1, x2) blocks then
                let
                    h = blockPosH b blocks + blockLength b blocks
                in
                    case blocksOn b blockPlacements of
                        [] -> h
                        blocksOnb ->
                            max h (maximum (fmap lowestEmptyHeightLevelInxRange' blocksOnb))
            else 0


-- | Returns the height of the block stack containing b.
blockStackHeight :: BlockId -> BlockSet -> Float
blockStackHeight b (blocks, blockPlacements) = blockStackHeight' (blockStackBase b blockPlacements)
    where
        blockStackHeight' b' =
            case blocksOn b' blockPlacements of
                [] -> blockPosH b' blocks + blockLength b' blocks
                blocksOnb' ->
                    maximum [blockStackHeight' aBlockOnb' | aBlockOnb' <- blocksOnb']


-- | Returns the lowest block (i.e base block) that is below and in the same stack as b. 
--   This block equals b if b is on the table
blockStackBase :: BlockId -> BlockPlacements -> BlockId
blockStackBase b blockPlacements =
    if blockBelowb == "B0" then b
    else blockStackBase blockBelowb blockPlacements
    where
        (_, blockBelowb) = Set.elemAt (blockIdx b) blockPlacements


-- | Returns True if b' is above b i.e directly on b or on a block
--   that is above b
blockIsAbove :: BlockId -> BlockId -> Blocks -> Bool
blockIsAbove b b' blocks =
    let
        blockb_x1 = blockPosX1 b blocks
        blockb_x2 = blockPosX2 b blocks
        h = blockPosH b blocks
        l = blockLength b blocks

        blockb'_x1 = blockPosX1 b' blocks
        blockb'_x2 = blockPosX2 b' blocks
        h' = blockPosH b' blocks
    in
        (blockb'_x1 >= blockb_x1) && (blockb'_x2 <= blockb_x2) && (h' >= h + l)


-- | Returns a list of blocks directly on block b
blocksOn :: BlockId -> BlockPlacements -> [BlockId]
blocksOn "B0" = baseBlocks
blocksOn b = Set.foldl foldFunc []
    where
        foldFunc = \blocksOnbSoFar (b1, b2) -> if b2 == b then b1:blocksOnbSoFar else blocksOnbSoFar


-- | Returns a list of all base blocks i.e blocks directly on the table
baseBlocks :: BlockPlacements -> [BlockId]
baseBlocks = Set.foldl foldFunc []
    where
        foldFunc = \baseBlocksSoFar (b1, b2) -> if b2 == "B0" then b1:baseBlocksSoFar else baseBlocksSoFar


-- | Returns True if block b intersects the xrange [min(x1, x2), max(x1, x2)]
--   i.e the intervals [blockb_x1, blockb_x2] and [min(x1, x2), max(x1, x2)] intersect 
blockIntersectsxRange :: BlockId -> (Float, Float) -> Blocks -> Bool
blockIntersectsxRange b (x1, x2) blocks =
    let
        blockb_x1 = blockPosX1 b blocks
        blockb_x2 = blockPosX2 b blocks
        x1' = min x1 x2
        x2' = max x1 x2
    in
        max x1' blockb_x1 < min x2' blockb_x2


-- | Returns True if placing b' on b creates a block stack whose height is more than the
--   the maximum height a block stack's allowed to reach
createsInvalidBlockStack :: BlockId -> BlockId -> Blocks -> Bool
createsInvalidBlockStack b b' blocks = 
    blockPosH b blocks + blockLength b blocks + blockLength b' blocks > maxBlockStackHeight


manhattanDistance :: Position -> Position -> Float
manhattanDistance (x1, h1) (x2, h2) = abs (x1 - x2) + abs (h1 - h2) 
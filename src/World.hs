{-# LANGUAGE TemplateHaskell #-}

module World where

import qualified Data.Map as Map
import qualified Data.Set as Set
import Data.List (isPrefixOf)
import Data.Sequence (Seq)
import Control.Lens.TH ( makeLenses )
import Control.Lens.Operators ( (^.) )
import Control.Lens.Tuple ( Field1(_1), Field2(_2) )
import Control.Concurrent.Async

import System.Console.Haskeline
import Graphics.Gloss


type Position = (Float, Float) -- (xCoordinate, heighAboveTheTable)

type BlockId = String

data Block
        = Block
        { _blockId       :: BlockId
        , _blockPos      :: Position
        , _blockSize     :: (Float, Float)
        , _blockColor    :: Color }

type BlockPlacements = Set.Set (BlockId, BlockId)
type Blocks = (Map.Map BlockId Block)
type BlockSet = (Blocks, BlockPlacements)


data Task = PUT_ON BlockId BlockId Position | READY_ARM_ON BlockId

data Maneuver = MoveToPosition Position | Grasp BlockId | UnGraspOn BlockId

type TaskSeq = Seq Task
type ManeuverSeq = Seq Maneuver

data Arm
        = Arm
        { _armPos                 :: Position
        , _graspedBlockObj        :: Maybe Block
        , _armTaskSeq             :: TaskSeq
        , _armManeuverSeq         :: ManeuverSeq
        , _isBusy                 :: Bool }

data World
        = World
        { _arm             :: Arm
        , _blockSet        :: BlockSet
        , _userInputBuffer :: Async String }

data UserCmd = PUTON BlockId BlockId | PUTON_TABLE BlockId deriving (Show)

makeLenses ''Block
makeLenses ''Arm
makeLenses ''World


instance Show Task where
    show (PUT_ON "B0" b1 pos) = "--> Put " ++ b1 ++ " on the table at position " ++ show pos
    show (PUT_ON b2 b1 _) = "--> Put " ++ b1 ++ " on " ++ b2 
    show (READY_ARM_ON b) = "--> Position the arm on " ++ b


drawWorld :: World -> IO Picture
drawWorld world = return $ Pictures
    [
        drawTable
    ,   drawHorizontalShaft
    ,   drawArm (world ^. arm)
    ,   pictures $ fmap (translate 0 (- worldHeight / 2) . drawBlock) (Map.elems blocks)
    ]
    where
        blocks = world ^. (blockSet . _1)


drawArm :: Arm -> Picture
drawArm rArm = translate x yCorrection $ pictures
    [
        rectangleUpperSolid 10 (armLength - bLength)
    ,   translate 0 (15 / 2) $ rectangleSolid 30 15
    ,   translate 0 (- bLength / 2)
        $ pictures
            [
                rectangleWire bWidth bLength
            ,   color (withAlpha 0.6 bColor) $ rectangleSolid bWidth bLength
            ,   translate (-7) 0 $ scale 0.1 0.1 $ text bText
            ]
    ]
    where
        (x, h) = rArm ^. armPos
        gblockObj = rArm ^. graspedBlockObj
        (bWidth, bLength) = maybe (0, 0) _blockSize gblockObj
        bColor = maybe white _blockColor gblockObj
        bText = maybe "" _blockId gblockObj 
        armLength = worldHeight - h
        yCorrection = (worldHeight / 2) - (armLength - bLength)


drawBlock :: Block -> Picture
drawBlock block = translate x (h + l / 2) $ pictures
    [
        rectangleWire w l
    ,   color (withAlpha 0.6 (block ^. blockColor)) $ rectangleSolid w l
    ,   translate (-7) 0 $ scale 0.1 0.1 $ text (block ^. blockId)
    ]
    where
        (x, h) = block ^. blockPos
        (w, l) = block ^. blockSize


drawTable :: Picture
drawTable = 
    translate 0 (- worldHeight / 2)
    $ color (makeColor 0.552 0.431 0.388 1) 
    $ translate 0 (- l / 2) $ rectangleSolid w l
    where
        w = 600
        l = 16

drawHorizontalShaft :: Picture
drawHorizontalShaft = 
    translate 0 (worldHeight / 2)
    $ translate 0 (l / 2) $ rectangleSolid w l
    where
        w = 600
        l = 10


-- | Tis' the distance from the table top vertical shaft to which the robot arm
--   is anchored
worldHeight :: Float
worldHeight = 600


maxBlockStackHeight :: Float
maxBlockStackHeight = 500


readCmdLineInput :: IO String
readCmdLineInput = runInputT cmdLineSettings readCmdLineInput'
    where
        readCmdLineInput' :: InputT IO String
        readCmdLineInput' = do
            minput <- getInputLine "Cmd> "
            case minput of
                Nothing -> return "quit" -- CTRL-D - meaning we should quit.
                Just input -> 
                    if null (words input) then readCmdLineInput'
                    else return input
        


cmdLineSettings :: Settings IO
cmdLineSettings = Settings 
    { historyFile = Just ".cmdHistory.txt"
    , complete = completeWord Nothing " \t" $ return . completeCmd
    , autoAddHistory = True }


completeCmd :: String -> [Completion]
completeCmd str = simpleCompletion <$> filter (str `isPrefixOf`) cmds
    where
        cmds = ["help", "quit", "clear", "reset", "PUTON", "PUTON_TABLE"]
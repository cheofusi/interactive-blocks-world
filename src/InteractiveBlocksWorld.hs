{-# OPTIONS_GHC -Wno-incomplete-patterns #-}

module InteractiveBlocksWorld where

import qualified Data.Map as Map
import qualified Data.Set as Set
import qualified Data.Sequence as Seq
import Data.List
import Data.Char (isSpace)
import Control.Lens.Operators ( (^.), (.~), (&) )
import Control.Lens.Tuple ( Field1(_1), Field2(_2) )
import Control.Monad (unless)

import System.Exit (exitWith, exitSuccess, exitFailure)
import Control.Concurrent.Async
import qualified System.Console.ANSI as ANSI

import Graphics.Gloss
import Graphics.Gloss.Interface.IO.Interact
import Graphics.Gloss.Geometry.Line
import Graphics.Gloss.Interface.IO.Game (playIO)

import World
import RobotArm
import RobotBrain


-- On each call to the updateWorld function, we check for input from the prompt if the robot is idle,
-- validate that input, and pass it to the solver which returns a list of tasks for the robot. 
-- If the robot isn't idle, then it is executing a task sequence. So we simply update its current state.


main = do
    ANSI.clearFromCursorToScreenBeginning
    ANSI.setCursorPosition 0 0
    asyncInput <- async readCmdLineInput
    playIO windowDisplay background 20 (initWorld asyncInput) drawWorld handleInput updateWorld


windowDisplay :: Display
windowDisplay = InWindow "Interactive Blocks World" (800, 640) (500, 10)


background :: Color
background = makeColor 1 1 1 0.5


initWorld :: Async String -> World
initWorld asyncInput = World
    { _arm = initRobotArm
    , _blockSet = (initBlocks, initBlockPlacements)
    , _userInputBuffer = asyncInput }


initRobotArm :: Arm
initRobotArm = Arm
    { _armPos = (0, 300)
    , _graspedBlockObj = Nothing
    , _armTaskSeq = Seq.Empty
    , _armManeuverSeq = Seq.Empty
    , _isBusy = False }


initBlocks :: Blocks
initBlocks = -- Attention tinkerer! If you're editing a block's attributes make sure to take into account
             -- the table size and the world height, all defined in World.hs
    Map.fromList [
            ("B1", Block "B1" (-70, 0) (80, 120) red)
        ,   ("B2", Block "B2" (-90, 120) (40, 60) blue)
        ,   ("B3", Block "B3" (40, 0) (40, 120) blue)
        ,   ("B4", Block "B4" (40, 120) (40, 60) yellow)
        ,   ("B5", Block "B5" (100, 0) (80, 60) green)
        ,   ("B6", Block "B6" (80, 60) (40, 60) green)
        ,   ("B7", Block "B7" (80, 120) (40, 60) red)
        ,   ("B8", Block "B8" (120, 60) (40, 120) yellow)
        ]


initBlockPlacements :: BlockPlacements
initBlockPlacements =
    Set.fromList [
            ("B1", "B0") -- This pair means the block B1 is on the block B0 (i.e the table)
        ,   ("B2", "B1")
        ,   ("B3", "B0")
        ,   ("B4", "B3")
        ,   ("B5", "B0")
        ,   ("B6", "B5")
        ,   ("B7", "B6")
        ,   ("B8", "B5")
        ]


updateWorld :: Float -> World -> IO World
updateWorld dt world =
    if armIsIdle (world ^. arm) then
        do
            res <- poll (world ^. userInputBuffer)
            case res of
              Nothing -> return world -- buffer is empty. Nothing to read
              Just (Left e) -> exitFailure -- buffer threw exception. exit
              Just (Right input) -> processCmd (trim input) dt world
                        
    else
        updateArm dt world
    where
        trim = dropWhileEnd isSpace . dropWhile isSpace


handleInput :: Event -> World -> IO World
handleInput _ = return


type Tokens = [String]
type ErrorStr = String


processCmd :: String -> Float -> World -> IO World
processCmd cmd dt world 
    | cmd == "quit" = exitSuccess
    | cmd == "help" = showHelpMsg >> resetPrompt world
    | cmd == "clear" = clearScreen >> resetPrompt world
    | cmd == "reset" = async readCmdLineInput >>= \asyncInput -> return $ initWorld asyncInput
    | otherwise = 
        case parseUserCmd cmd (world ^. (blockSet . _1)) of
            Left errorStr -> do
                ANSI.setSGR [ANSI.SetColor ANSI.Foreground ANSI.Dull ANSI.Red]
                putStrLn errorStr
                resetDefaultColor
                resetPrompt world

            Right uCmd ->
                case solveForInput uCmd (world ^. blockSet) of
                    Left (msg, taskSeq) -> do -- input cmd was impossible to satisfy
                        ANSI.setSGR [ANSI.SetColor ANSI.Foreground ANSI.Vivid ANSI.Red]
                        putStrLn msg
                        resetDefaultColor
                        unless (null taskSeq) $ do
                            putStrLn "Before failing, the task Sequence was: "
                            ANSI.setSGR [ANSI.SetColor ANSI.Foreground ANSI.Dull ANSI.Green]
                            mapM_ print taskSeq
                            resetDefaultColor

                        resetPrompt world

                    Right (_, taskSeq) -> do
                        let blockToStartOn = case Seq.index taskSeq 0 of
                                PUT_ON _ b _ -> b
                                --solveForInput never generates a READY_ARM_ON task 

                        let task1 = READY_ARM_ON blockToStartOn
                        let taskSeq' = task1 Seq.<| taskSeq

                        putStrLn $ 
                                "Task sequence for executing the command " 
                            ++  ANSI.setSGRCode [ANSI.SetColor ANSI.Foreground ANSI.Vivid ANSI.Cyan]
                            ++  show uCmd 
                            ++  ": " 

                        ANSI.setSGR [ANSI.SetColor ANSI.Foreground ANSI.Dull ANSI.Green]
                        mapM_ print taskSeq'
                        resetDefaultColor

                        putStrLn "\nExecuting command.."
                        return $ world
                            & arm . armTaskSeq .~ taskSeq'
                            & arm . isBusy .~ True


parseUserCmd :: String -> Blocks -> Either ErrorStr UserCmd
parseUserCmd inputStr blocks = do
    let inputTokens = words inputStr
    case head inputTokens of
        "PUTON" ->
            if length (tail inputTokens) /= 2 then
                Left "invalid # of args for PUTON command. Should be 2"
            else
                do
                (b2, inputTokens') <- parseBlockId (tail inputTokens) blocks
                (b1, inputTokens'') <- parseBlockId inputTokens' blocks
                if b1 == b2 then
                    Left $ "cannot place block " ++ b1 ++ " on itself"
                else
                    Right $ PUTON b2 b1
        "PUTON_TABLE" ->
            if length (tail inputTokens) /= 1 then
                Left "invalid # of args for PUTON_TABLE command. Should be 1"
            else
                do
                (b, inputTokens') <- parseBlockId (tail inputTokens) blocks
                Right $ PUTON_TABLE b

        invCmd -> Left $ "invalid command: " ++ invCmd


parseBlockId :: Tokens -> Blocks -> Either ErrorStr (BlockId, Tokens)
parseBlockId (token:tokens) blocks =
    case Map.lookup token blocks of
        Nothing -> Left $ "No block named " ++ token
        Just _ -> Right (token, tokens)


clearScreen :: IO ()
clearScreen = do
    ANSI.clearFromCursorToScreenBeginning
    ANSI.setCursorPosition 0 0


showHelpMsg :: IO ()
showHelpMsg = do
    putStrLn "help               Show this help text"
    putStrLn "quit               Exit Interactive Blocks World"
    putStrLn "clear              Clear console output"
    putStrLn "reset              Reset the blocks and arm to their initial configuration"
    putStrLn "PUTON [By] [Bx]    Place the block Bx on the block By. By's width must be at least Bx's width"
    putStrLn "PUTON_TABLE [Bx]   Place the block Bx on the table"


resetPrompt :: World -> IO World
resetPrompt world = do
    asyncInput <- async readCmdLineInput
    return $ world & userInputBuffer .~ asyncInput


resetDefaultColor :: IO ()
resetDefaultColor = ANSI.setSGR [ANSI.Reset]


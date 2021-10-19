module Utils where

import qualified Data.Map as Map
import Control.Lens.Operators ( (^.) )
import Control.Lens.Tuple ( Field1(_1), Field2(_2) )

import World



armPosX :: Arm -> Float
armPosX rArm = rArm ^. (armPos . _1)


armPosH :: Arm -> Float
armPosH rArm = rArm ^. (armPos . _2)


blockPosition :: BlockId -> Blocks -> Position
blockPosition b blocks = blockObject b blocks ^. blockPos


blockPosX :: BlockId -> Blocks -> Float
blockPosX b blocks = blockObject b blocks ^. (blockPos . _1)


blockPosH :: BlockId -> Blocks -> Float
blockPosH b blocks = blockObject b blocks ^. (blockPos . _2)


blockWidth :: BlockId -> Blocks -> Float
blockWidth b blocks = blockObject b blocks ^. (blockSize . _1)


blockLength :: BlockId -> Blocks -> Float
blockLength b blocks = blockObject b blocks ^. (blockSize . _2)


blockPosX1 :: BlockId -> Blocks -> Float
blockPosX1 b blocks = blockPosX b blocks - (blockWidth b blocks / 2) 


blockPosX2 :: BlockId -> Blocks -> Float
blockPosX2 b blocks = blockPosX b blocks + (blockWidth b blocks / 2)  


-- | Returns the block object associated with the blockId b
blockObject :: BlockId -> Blocks -> Block
blockObject b blocks = snd $ Map.elemAt (blockIdx b) blocks


-- | Returns index of block b as it is ordered in the Blocks map & BlockPlacements set. 
--   ex. with b3 this function returns 2
blockIdx :: BlockId -> Int
blockIdx b = (read (drop 1 b) :: Int) - 1
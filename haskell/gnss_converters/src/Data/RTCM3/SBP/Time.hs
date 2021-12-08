{-# OPTIONS  -fno-warn-overflowed-literals #-}
{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Time
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- SBP GPS Time helpers.

module Data.RTCM3.SBP.Time
  ( gpsLeapMillis
  , minuteMillis
  , hourMillis
  , dayMillis
  , weekMillis
  , toWn
  , toStartDate
  , toTow
  , currentGpsTime
  , gpsRolloverGpsTime
  , glonassRolloverGpsTime
  , glonassRolloverGpsTime'
  , beidouRolloverGpsTime
  , modifyIORefM
  , toGpsTime
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.IORef
import Data.RTCM3.SBP.Types
import Data.Time
import Data.Time.Calendar.WeekDate
import Data.Word
import SwiftNav.SBP

-- | Beginning of GPS time.
--
gpsEpoch :: Day
gpsEpoch = fromGregorian 1980 1 6

-- | Number of GPS leap seconds.
--
gpsLeapSeconds :: Integer
gpsLeapSeconds = 18

-- | Number of GPS leap milliseconds.
--
gpsLeapMillis :: Integer
gpsLeapMillis = 1000 * gpsLeapSeconds

-- | Number of BeiDou offset seconds.
--
beidouOffsetSeconds :: Integer
beidouOffsetSeconds = 14

-- | Number of BeiDou offet milliseconds.
--
beidouOffsetMillis :: Integer
beidouOffsetMillis = 1000 * beidouOffsetSeconds

-- | Minute seconds
--
minuteSeconds :: Integer
minuteSeconds = 60

-- | Minute milliseconds
--
minuteMillis :: Integer
minuteMillis = 1000 * minuteSeconds

-- | Hour seconds
--
hourSeconds :: Integer
hourSeconds = 60 * 60

-- | Hour milliseconds
--
hourMillis :: Integer
hourMillis = 1000 * hourSeconds

-- | Day seconds
--
daySeconds :: Integer
daySeconds = 24 * hourSeconds

-- | Day milliseconds
--
dayMillis :: Integer
dayMillis = 1000 * daySeconds

-- | Number of seconds in a week.
--
weekSeconds :: Integer
weekSeconds = 7 * daySeconds

-- | Number of milliseconds in a week.
--
weekMillis :: Integer
weekMillis = 1000 * weekSeconds

-- | Produce GPS week number from a GPS UTC time.
--
toWn :: UTCTime -> Word16
toWn t = fromIntegral weeks
  where
    days  = diffDays (utctDay t) gpsEpoch
    weeks = days `div` 7

-- | Find the start of the GPS week, which is Sunday.
--
toStartDate :: (Integer, Int, Int) -> (Integer, Int, Int)
toStartDate (year, week, day)
  | day == 7  = (year,   week,   7)
  | week == 1 = (year-1, 52,     7)
  | otherwise = (year,   week-1, 7)

-- | Generate the start of the GPS week UTC time.
--
fromStartDate :: UTCTime -> UTCTime
fromStartDate t = UTCTime (fromWeekDate year week day) 0
  where
    (year, week, day) = toStartDate $ toWeekDate $ utctDay t

-- | Produce GPS time of week from a GPS UTC time.
--
toTow :: UTCTime -> Word32
toTow t = floor $ 1000 * diffUTCTime t (fromStartDate t)

-- | Get current GPS time.
--
currentGpsTime :: MonadIO m => m GpsTime
currentGpsTime = do
  t <- liftIO $ addUTCTime (fromIntegral gpsLeapSeconds) <$> getCurrentTime
  pure $ GpsTime (toTow t) 0 (toWn t)

-- | Update GPS time based on GPS time of week, handling week rollover.
--
gpsRolloverGpsTime :: Word32 -> GpsTime -> GpsTime
gpsRolloverGpsTime tow t = t & gpsTime_tow .~ tow & rollover
  where
    rollover
      | increment = gpsTime_wn +~ 1
      | decrement = gpsTime_wn +~ -1
      | otherwise = gpsTime_wn +~ 0
    new       = fromIntegral tow
    old       = fromIntegral (t ^. gpsTime_tow)
    increment = old > new && old - new > weekMillis `div` 2
    decrement = new > old && new - old > weekMillis `div` 2

-- | Update GPS time based on GLONASS epoch, handling week rollover.
--
glonassRolloverGpsTime :: Word32 -> GpsTime -> GpsTime
glonassRolloverGpsTime epoch t = gpsRolloverGpsTime tow t
  where
    epoch' = fromIntegral epoch - 3 * hourMillis + gpsLeapMillis
    epoch''
      | epoch' < 0 = epoch' + dayMillis
      | otherwise  = epoch'
    dow = fromIntegral (t ^. gpsTime_tow) `div` dayMillis
    tod = fromIntegral (t ^. gpsTime_tow) - dow * dayMillis
    dow'
      | increment = dow + 1 `mod` 7
      | decrement = dow - 1 `mod` 7
      | otherwise = dow
    increment = epoch'' > tod && epoch'' - tod > dayMillis `div` 2
    decrement = tod > epoch'' && tod - epoch'' > dayMillis `div` 2
    tow       = fromIntegral $ dow' * dayMillis + epoch''

-- | Update GPS time based on MSM GLONASS epoch, handling week rollover.
--
glonassRolloverGpsTime' :: Word32 -> GpsTime -> GpsTime
glonassRolloverGpsTime' epoch = glonassRolloverGpsTime epoch'
  where
    epoch' = epoch .&. 134217727

-- | Update GPS time based on MSM BeiDou epoch, handling week rollover.
--
beidouRolloverGpsTime :: Word32 -> GpsTime -> GpsTime
beidouRolloverGpsTime epoch = gpsRolloverGpsTime tow
  where
    epoch' = fromIntegral epoch + beidouOffsetMillis
    epoch''
      | epoch' >= dayMillis = epoch' - dayMillis
      | otherwise           = epoch'
    tow = fromIntegral epoch''

-- | Monadic IORef modify.
--
modifyIORefM :: MonadIO m => IORef a -> (a -> m (a, b)) -> m b
modifyIORefM ref f = do
  x      <- liftIO $ readIORef ref
  (y, z) <- f x
  liftIO $ writeIORef ref y
  pure z

-- | Update and convert stored and incoming GPS times.
--
toGpsTime :: MonadStore e m => Word16 -> (GpsTime -> GpsTime) -> m (GpsTime, GpsTime)
toGpsTime station rollover = do
  timeMap <- view storeGpsTimeMap
  modifyIORefM timeMap $ \timeMap' -> do
    time <- view storeCurrentGpsTime
    t    <- maybe (liftIO time) pure (timeMap' ^. at station)
    let t' = rollover t
    pure (timeMap' & at station ?~ t', (t, t'))

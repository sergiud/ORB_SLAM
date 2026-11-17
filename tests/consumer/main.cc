// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Includes every installed header and uses the library through them.

#include <cstdlib>
#include <print>

#include <Converter.h>
#include <Frame.h>
#include <FramePublisher.h>
#include <Initializer.h>
#include <KeyFrame.h>
#include <KeyFrameDatabase.h>
#include <LocalMapping.h>
#include <LoopClosing.h>
#include <Map.h>
#include <MapPoint.h>
#include <ORBVocabulary.h>
#include <ORBextractor.h>
#include <ORBmatcher.h>
#include <ORB_SLAM_export.h>
#include <Optimizer.h>
#include <PnPsolver.h>
#include <Sampling.h>
#include <Sim3Solver.h>
#include <Tracking.h>

int main()
{
    ORB_SLAM::Map map;
    ORB_SLAM::ORBVocabulary vocabulary;
    ORB_SLAM::KeyFrameDatabase database{vocabulary};
    database.clear();

    if (map.KeyFramesInMap() != 0) {
        std::println(stderr, "an empty map holds keyframes");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

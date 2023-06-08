// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file HoughTransformLaneDetector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief hough transform lane detector class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <numeric>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"
namespace Xycar {

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"].as<PREC>();
    mHoughThreshold = config["HOUGH"]["THRESHOLD"].as<int32_t>();
    mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"].as<int32_t>();
    mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::pair<PREC, PREC> HoughTransformLaneDetector<PREC>::getLineParameters(const Lines& lines, const Indices& lineIndices)
{
    uint32_t numLines = static_cast<uint32_t>(lineIndices.size());
    if (numLines == 0)
        return { 0.0f, 0.0f };

    int32_t xSum = 0;
    int32_t ySum = 0;
    PREC mSum = 0.0f;
    for (const auto lineIndex : lineIndices)
    {
        int32_t x1 = lines[lineIndex][HoughIndex::x1];
        int32_t y1 = lines[lineIndex][HoughIndex::y1];
        int32_t x2 = lines[lineIndex][HoughIndex::x2];
        int32_t y2 = lines[lineIndex][HoughIndex::y2];
        xSum += x1 + x2;
        ySum += y1 + y2;
        mSum += static_cast<PREC>((y2 - y1)) / (x2 - x1);
    }

    PREC xAverage = static_cast<PREC>(xSum) / (numLines * 2);
    PREC yAverage = static_cast<PREC>(ySum) / (numLines * 2);
    PREC m = mSum / numLines;
    PREC b = yAverage - m * xAverage;

    return { m, b };
}

template <typename PREC>
int32_t HoughTransformLaneDetector<PREC>::getLinePositionX(const Lines& lines, const Indices& lineIndices, Direction direction)
{
    const auto [m, b] = getLineParameters(lines, lineIndices);

    if (std::abs(m) <= std::numeric_limits<PREC>::epsilon() && std::abs(b) <= std::numeric_limits<PREC>::epsilon())
    {
        if (direction == Direction::LEFT)
            return 0.0f;
        else if (direction == Direction::RIGHT)
            return static_cast<PREC>(mImageWidth);
    }

    PREC y = static_cast<PREC>(mROIHeight) * 0.5f;
    std::cout << "slope: " << m << std::endl;
    return std::round((y - b) / m);
}

template <typename PREC>
std::pair<Indices, Indices> HoughTransformLaneDetector<PREC>::divideLines(const Lines& lines)
{
    Indices leftLineIndices;
    Indices rightLineIndices;
    uint32_t linesSize = static_cast<uint32_t>(lines.size());
    leftLineIndices.reserve(linesSize);
    rightLineIndices.reserve(linesSize);
    PREC slope = 0.0f;
    PREC leftLineSumX = 0.0f;
    PREC rightLineSumX = 0.0f;

    for (uint32_t i = 0; i < linesSize; ++i)
    {
        const auto& line = lines[i];

        int32_t x1 = line[HoughIndex::x1];
        int32_t y1 = line[HoughIndex::y1];
        int32_t x2 = line[HoughIndex::x2];
        int32_t y2 = line[HoughIndex::y2];

        if (x2 - x1 == 0)
            slope = 0.0f;
        else
            slope = static_cast<PREC>(y2 - y1) / (x2 - x1);

        if (-mHoughLineSlopeRange <= slope && slope <= -0.28f)
        {//-10  0  left
            leftLineSumX += static_cast<PREC>(x1 + x2) * 0.5f;
            leftLineIndices.emplace_back(i);
        }/*
        else if(0.0f > slope && slope > -0.28f){//left curve
            leftLineSumX += static_cast<PREC>(x1 * 0.8f + x2 * 0.2f);
            leftLineIndices.emplace_back(i);
            
        }
        else if(0.0f < slope && slope < 0.28f){
            rightLineSumX += static_cast<PREC>(x1 * 0.2f + x2 * 0.8f);
            rightLineIndices.emplace_back(i);
        }*/
        else if (0.28f <= slope && slope <= mHoughLineSlopeRange)
        {// 0 < 10  right
            rightLineSumX += static_cast<PREC>(x1 + x2) * 0.5f;
            rightLineIndices.emplace_back(i);
        }
    }

    auto numLeftLines = static_cast<uint32_t>(leftLineIndices.size());
    auto numRightLines = static_cast<uint32_t>(rightLineIndices.size());

    if (numLeftLines != 0 && numRightLines != 0)
    {
        auto leftAverageX = static_cast<PREC>(leftLineSumX / numLeftLines);
        auto rightAverageX = static_cast<PREC>(rightLineSumX / numRightLines);
        if (leftAverageX > rightAverageX)
        {
            leftLineIndices.clear();
            rightLineIndices.clear();
            std::cout << "------Invalid Path!------" << std::endl;
        }
    }

    return { leftLineIndices, rightLineIndices };
    
}

template <typename PREC>
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
{
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    cv::Mat cannyImage;
    cv::Canny(grayImage, cannyImage, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);
    cv::Mat ROI = cannyImage(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));
    Lines allLines;
    cv::HoughLinesP(ROI, allLines, kHoughRho, kHoughTheta, mHoughThreshold, mHoughMinLineLength, mHoughMaxLineGap);

    if (mDebugging)
        image.copyTo(mDebugFrame);

    if (allLines.empty())
        return { 0, mImageWidth };

    const auto [leftLineIndices, rightLineIndices] = divideLines(allLines);

    auto leftPositionX = getLinePositionX(allLines, leftLineIndices, Direction::LEFT);
    auto rightPositionX = getLinePositionX(allLines, rightLineIndices, Direction::RIGHT);


    return { leftPositionX, rightPositionX };
}

/*template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices)
{
    auto draw = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();

            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, { b, g, r }, kDebugLineWidth);
        }
    };

    draw(lines, leftLineIndices);
    draw(lines, rightLineIndices);
}*/

/*template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(leftPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(rightPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(estimatedPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(estimatedPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kRed, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(mImageWidth / 2 - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(mImageWidth / 2 + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kBlue, kDebugLineWidth);
}*/

void Decide_angle(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    
}

template class HoughTransformLaneDetector<float>;
template class HoughTransformLaneDetector<double>;
} // namespace Xycar
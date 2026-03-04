/*
 VisionProcessor.cpp
 Implementation of the ARGUS vision safety evaluator.
 
 This file contains the logic for ArUco marker detection and
 safety evaluation. Each method is designed to be deterministic,
 bounded in execution time and free of blocking calls.
 
 Processing is intentionally split across methods to keep each
 function focused on a single responsibility, making the logic
 easier to test, debug and reason about.
*/

#include "VisionProcessor.hpp"

// Constructor
// All one-time setup happens here — dictionary creation and detector initialisation 
// so that process() itself remains fast and its execution time predictable.
//

VisionProcessor::VisionProcessor(const VisionConfig& config)
    : config_(config),

      // Initialise the ArUco dictionary once at construction.
      // DICT_6X6_250 provides 250 possible marker IDs with 6x6 bit
      // encoding — good error correction and reliable detection at
      // typical camera distances. Chosen over larger dictionaries
      // to maximise inter-marker distance and minimise false positives,
      // which is critical in a safety-monitoring context.
      dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),

      // Construct the detector with default parameters and the dictionary
      // above. DetectorParameters uses its defaults unless overridden.
      // Constructed here rather than per-frame to avoid repeated heap
      // allocation inside process(), keeping latency bounded.
      detector_(dictionary_, detectorParams_)
{
    // hasPrevious_ is initialised to false in the header (= false).
    // No previous centroid exists until the first successful detection.
}
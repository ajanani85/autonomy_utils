#ifndef AUTONOMY_UTILS_LIB_TRANSFORM_HANDLER_H_
#define AUTONOMY_UTILS_LIB_TRANSFORM_HANDLER_H_

#include <autonomy_utils/lib/transformer.h>
#include <autonomy_utils/lib/ros2_core.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace ros2
{
/**
 * @class TransformHandler
 * @brief A utility class for handling transformations between two frames in ROS 2.
 * 
 * The TransformHandler class provides an interface to manage and retrieve transformations
 * between a source frame and a target frame using a shared Transformer instance. It ensures
 * that transformations are handled efficiently and provides utility functions for resetting
 * and printing transformation data.
 * 
 * @note This class is designed to work with ROS 2 and assumes the use of geometry_msgs for
 * transformation data.
 */
class TransformHandler {
public:
    /**
     * @brief Constructs a TransformHandler object.
     * 
     * Initializes a TransformHandler instance with a shared pointer to a ros2::Transformer
     * object and two strings representing the source and target frames. The constructor is
     * marked as explicit to prevent unintended implicit conversions.
     * 
     * @param transformer A shared pointer to a ros2::Transformer instance used for managing transformations.
     * @param source The name of the source frame.
     * @param target The name of the target frame.
     */
    explicit TransformHandler(std::shared_ptr<ros2::Transformer> transformer, const std::string &source, const std::string &target);

    /**
     * @brief Destructor for the TransformHandler class.
     * 
     * Cleans up resources used by the TransformHandler instance.
     */
    ~TransformHandler();

    /**
     * @brief Retrieves the current transformation.
     * 
     * Returns a reference to the stored TransformStamped object representing the transformation
     * between the source and target frames.
     * 
     * @return A reference to the geometry_msgs::msg::TransformStamped object.
     */
    geometry_msgs::msg::TransformStamped &getTransform();

    /**
     * @brief Retrieves the current transformation and stores it in a TransformStamped object.
     * This function checks if the transformation is set and retrieves it if available.
     * @param tf_stamped A reference to a geometry_msgs::msg::TransformStamped object where the transformation will be stored.
     * @return True if the transformation was successfully retrieved, false otherwise.
     */
    bool getTransform(geometry_msgs::msg::TransformStamped &tf_stamped);


    /**
     * @brief Resets the transformation state.
     * 
     * Clears the current transformation data and marks the transformation as unset.
     */
    void reset();

    /**
     * @brief Prints the transformation data with a custom message.
     * 
     * Outputs the current transformation data along with a user-provided message for debugging
     * or logging purposes.
     * 
     * @param msg A string message to be printed alongside the transformation data.
     */
    void print(const std::string &msg);

    /**
     * @brief Checks if the transformation is set.
     * 
     * Returns true if the transformation between the source and target frames
     * has been set, otherwise returns false.
     * 
     * @return True if the transformation is set, false otherwise.
     */
    bool isSet();

private:
    /**
     * @brief The name of the source frame.
     */
    const std::string source_;

    /**
     * @brief The name of the target frame.
     */
    const std::string target_;

    /**
     * @brief A shared pointer to the Transformer instance.
     * 
     * Used for managing transformations between frames.
     */
    std::shared_ptr<ros2::Transformer> transformer_ = nullptr;

    /**
     * @brief Indicates whether the transformation is set.
     */
    bool is_set_ {false};

    /**
     * @brief Stores the current transformation data.
     */
    geometry_msgs::msg::TransformStamped tf_stamped_;
};

}

#endif /*AUTONOMY_UTILS_LIB_TRANSFORM_HELPER_H_ */
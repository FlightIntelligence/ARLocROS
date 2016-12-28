package arlocros;

import com.google.auto.value.AutoValue;
import org.ros.node.parameter.ParameterTree;

/** @author Hoang Tung Dinh */
@AutoValue
public abstract class Parameter {

  protected Parameter() {}

  public abstract String patternDirectory();

  public abstract String markerFrameName();

  public abstract String cameraFrameName();

  public abstract String cameraImageTopic();

  public abstract String cameraInfoTopic();

  public abstract String markerConfigFile();

  public abstract boolean badPoseReject();

  public abstract String fusedPoseTopicName();

  public abstract String markerPoseTopicName();

  public abstract boolean visualization();

  public abstract double blackWhiteContrastLevel();

  public abstract boolean invertBlackWhiteColor();

  public static Parameter createFrom(ParameterTree parameterTree) {
    return builder()
        .patternDirectory(parameterTree.getString("/pattern_dir"))
        .markerConfigFile(parameterTree.getString("/marker_config_file"))
        .markerFrameName(parameterTree.getString("/marker_frame_name"))
        .cameraFrameName(parameterTree.getString("/camera_frame_name"))
        .cameraImageTopic(parameterTree.getString("/camera_image_topic"))
        .cameraInfoTopic(parameterTree.getString("/camera_info_topic"))
        .badPoseReject(parameterTree.getBoolean("/bad_pose_reject"))
        .fusedPoseTopicName(parameterTree.getString("/fused_pose_topic_name"))
        .markerPoseTopicName(parameterTree.getString("/marker_pose_topic_name"))
        .visualization(parameterTree.getBoolean("/visualization"))
        .blackWhiteContrastLevel(parameterTree.getDouble("/black_white_contrast_level"))
        .invertBlackWhiteColor(parameterTree.getBoolean("/invert_black_white_color"))
        .build();
  }

  public static Builder builder() {
    return new AutoValue_Parameter.Builder();
  }

  @AutoValue.Builder
  public abstract static class Builder {
    public abstract Builder patternDirectory(String value);

    public abstract Builder markerFrameName(String value);

    public abstract Builder cameraFrameName(String value);

    public abstract Builder cameraImageTopic(String value);

    public abstract Builder cameraInfoTopic(String value);

    public abstract Builder markerConfigFile(String value);

    public abstract Builder badPoseReject(boolean value);

    public abstract Builder fusedPoseTopicName(String value);

    public abstract Builder markerPoseTopicName(String value);

    public abstract Builder visualization(boolean value);

    public abstract Builder blackWhiteContrastLevel(double value);

    public abstract Builder invertBlackWhiteColor(boolean value);

    public abstract Parameter build();
  }
}

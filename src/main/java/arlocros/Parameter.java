package arlocros;

import com.google.auto.value.AutoValue;
import org.ros.node.ConnectedNode;
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

  public abstract int filterBlockSize();

  public abstract double subtractedConstant();

  public abstract boolean invertBlackWhiteColor();

  public abstract int instanceId();

  public abstract String heartbeatTopicName();

  public static Parameter createFrom(ConnectedNode connectedNode) {
    final ParameterTree parameterTree = connectedNode.getParameterTree();
    final String nodeName = connectedNode.getName().toString();
    return builder().patternDirectory(parameterTree.getString("/pattern_dir"))
        .markerConfigFile(parameterTree.getString("/marker_config_file"))
        .markerFrameName(parameterTree.getString("/marker_frame_name"))
        .cameraFrameName(parameterTree.getString("/camera_frame_name"))
        .cameraImageTopic(parameterTree.getString("/camera_image_topic"))
        .cameraInfoTopic(parameterTree.getString("/camera_info_topic"))
        .badPoseReject(parameterTree.getBoolean("/bad_pose_reject"))
        .fusedPoseTopicName(parameterTree.getString("/fused_pose_topic_name"))
        .markerPoseTopicName(parameterTree.getString("/marker_pose_topic_name"))
        .visualization(parameterTree.getBoolean("/visualization"))
        .filterBlockSize(parameterTree.getInteger("/filter_block_size"))
        .subtractedConstant(parameterTree.getDouble("/subtracted_constant"))
        .invertBlackWhiteColor(parameterTree.getBoolean("/invert_black_white_color"))
        .instanceId(parameterTree.getInteger("/" + nodeName + "/instance_id"))
        .heartbeatTopicName(parameterTree.getString("/heartbeat_topic_name"))
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

    public abstract Builder filterBlockSize(int value);

    public abstract Builder subtractedConstant(double value);

    public abstract Builder invertBlackWhiteColor(boolean value);

    public abstract Builder instanceId(int value);

    public abstract Builder heartbeatTopicName(String value);

    public abstract Parameter build();
  }
}

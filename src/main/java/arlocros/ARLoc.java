package arlocros;

import geometry_msgs.PoseStamped;
import nav_msgs.Odometry;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import std_msgs.Int32;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/** @author Hoang Tung Dinh */
public class ARLoc extends AbstractNodeMain {

  private static final Logger logger = LoggerFactory.getLogger(ARLoc.class);

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/imshow");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    try {
      final Parameter parameter = Parameter.createFrom(connectedNode);

      final Publisher<Int32> heartbeatPublisher = connectedNode.newPublisher(
          parameter.heartbeatTopicName(), Int32._TYPE);

      Executors.newSingleThreadScheduledExecutor().scheduleAtFixedRate(() -> {
        final Int32 heartbeatMessage = heartbeatPublisher.newMessage();
        heartbeatMessage.setData(parameter.instanceId());
        heartbeatPublisher.publish(heartbeatMessage);
      }, 0, 100, TimeUnit.MILLISECONDS);

      final Publisher<PoseStamped> markerPosePubliser = connectedNode.newPublisher(
          parameter.markerPoseTopicName(), PoseStamped._TYPE);

      // add heartbeat publisher here

      HeartbeatMonitor heartbeatMonitor = null;
      if (parameter.instanceId() > 0) {
        // add message listener here
        heartbeatMonitor = HeartbeatMonitor.create(connectedNode, parameter.instanceId() - 1);
      }

      final MessagesSubscriberService<Int32> heartbeatMessageSubscriber =
          MessagesSubscriberService.create(
          connectedNode.<Int32>newSubscriber(parameter.heartbeatTopicName(), Int32._TYPE));

      if (heartbeatMonitor != null) {
        heartbeatMessageSubscriber.registerMessageObserver(heartbeatMonitor);
      }

      final PoseEstimator poseEstimator = ArMarkerPoseEstimator.create(connectedNode, parameter,
          markerPosePubliser, heartbeatMonitor);

      final BebopOdomVelocityEstimator velocityEstimator = BebopOdomVelocityEstimator.create();

      final MessagesSubscriberService<Odometry> odomSubscriber = MessagesSubscriberService.create(
          connectedNode.<Odometry>newSubscriber("/bebop/odom", Odometry._TYPE));
      odomSubscriber.registerMessageObserver(velocityEstimator);

      final Publisher<PoseStamped> fusedPosePublisher = connectedNode.newPublisher(
          parameter.fusedPoseTopicName(), PoseStamped._TYPE);

      final FusedLocalization fusedLocalization = FusedLocalization.create(poseEstimator,
          velocityEstimator, fusedPosePublisher, 40, connectedNode);
    } catch (Throwable e) {
      logger.error("Exception occurs", e);
    }
  }
}

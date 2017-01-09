package arlocros;

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import std_msgs.Int32;

import java.util.concurrent.atomic.AtomicReference;

/** @author Hoang Tung Dinh */
public final class HeartbeatMonitor implements MessageObserver<Int32> {
  private final ConnectedNode connectedNode;
  private final int idOfToBeMonitoredInstance;
  private final AtomicReference<Time> lastTimeReceivedMessage = new AtomicReference<>();

  private HeartbeatMonitor(final ConnectedNode connectedNode, int idOfToBeMonitoredInstance) {
    this.connectedNode = connectedNode;
    this.idOfToBeMonitoredInstance = idOfToBeMonitoredInstance;
  }

  public static HeartbeatMonitor create(ConnectedNode connectedNode,
      int idOfToBeMonitoredInstance) {
    return new HeartbeatMonitor(connectedNode, idOfToBeMonitoredInstance);
  }

  @Override
  public void onNewMessage(Int32 message) {
    if (message.getData() == idOfToBeMonitoredInstance) {
      lastTimeReceivedMessage.set(connectedNode.getCurrentTime());
    }
  }

  public Time getLastTimeReceivedMessage() {
    return lastTimeReceivedMessage.get();
  }
}

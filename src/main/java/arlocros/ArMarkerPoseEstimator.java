/*
 * Copyright (C) 2016 Marvin Ferber.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package arlocros;

import com.google.common.base.Optional;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import jp.nyatla.nyartoolkit.core.NyARException;
import org.apache.commons.lang.exception.ExceptionUtils;
import org.apache.commons.logging.Log;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import sensor_msgs.CameraInfo;
import tf2_msgs.TFMessage;

import javax.annotation.Nullable;
import java.io.FileNotFoundException;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Main Class of the ARLocROS Node setting up publishers and subscribers. Note the TF lookup
 * implementation based on Transformer by Lorenz Moesenlechner.
 */
public final class ArMarkerPoseEstimator implements PoseEstimator {

  private static final Logger logger = LoggerFactory.getLogger(ArMarkerPoseEstimator.class);

  private CameraParams camp;

  @Nullable private Parameter parameter;
  private MarkerConfig markerConfig;
  protected org.ros.rosjava_geometry.Transform last_pose;
  protected Time last_timestamp;

  protected boolean smoothing = true;
  private static Log log;

  private final Publisher<PoseStamped> posePublisher;

  private AtomicReference<PoseStamped> mostRecentPose;

  private ArMarkerPoseEstimator(
      final ConnectedNode connectedNode,
      Parameter parameter,
      Publisher<PoseStamped> posePublisher) {
    mostRecentPose = new AtomicReference<>();
    this.parameter = parameter;
    this.posePublisher = posePublisher;
    Executors.newSingleThreadExecutor()
        .submit(
            new Runnable() {
              @Override
              public void run() {
                start(connectedNode);
              }
            });
  }

  public static ArMarkerPoseEstimator create(
      ConnectedNode connectedNode, Parameter parameter, Publisher<PoseStamped> posePublisher) {
    return new ArMarkerPoseEstimator(connectedNode, parameter, posePublisher);
  }

  private void start(final ConnectedNode connectedNode) {
    // load OpenCV shared library
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

    // read configuration variables from the ROS Runtime (configured in the
    // launch file)
    log = connectedNode.getLog();

    // Read Marker Config
    markerConfig =
        MarkerConfig.createFromConfig(parameter.markerConfigFile(), parameter.patternDirectory());

    camp = getCameraInfo(connectedNode, parameter);

    // start to listen to transform messages in /tf in order to feed the
    // Transformer and lookup transforms
    final TransformationService transformationService = TransformationService.create(connectedNode);

    // Subscribe to Image
    Subscriber<sensor_msgs.Image> subscriberToImage =
        connectedNode.newSubscriber(parameter.cameraImageTopic(), sensor_msgs.Image._TYPE);

    ComputePose computePose = null;
    try {
      final Mat cameraMatrix = CameraParams.getCameraMatrix(camp);
      final MatOfDouble distCoeffs = CameraParams.getDistCoeffs(camp);
      computePose =
          ComputePose.create(
              markerConfig,
              new Size(camp.width(), camp.height()),
              cameraMatrix,
              distCoeffs,
              this.parameter.visualization());
    } catch (NyARException e) {
      logger.info("Cannot initialize ComputePose", e);
    } catch (FileNotFoundException e) {
      logger.info("Cannot find file when initialize ComputePose", e);
    }
    final ComputePose poseProcessor = computePose;
    final Publisher<tf2_msgs.TFMessage> tfPublisherCamToMarker = connectedNode.newPublisher("tf",
              tf2_msgs.TFMessage._TYPE);
    subscriberToImage.addMessageListener(
        new MessageListener<sensor_msgs.Image>() {

          @Override
          public void onNewMessage(sensor_msgs.Image message) {
            //
            if (!message.getEncoding().toLowerCase().equals("rgb8")) {
              log.error(
                  "Sorry, " + message.getEncoding() + " Image encoding is not supported! EXITING");
              System.exit(-1);
            }
            if (camp != null) {
              try {
                //
                final Mat image = Utils.matFromImage(message);
                // uncomment to add more contrast to the image
                final Mat thresholdedImage =
                    Utils.tresholdContrastBlackWhite(
                        image,
                        parameter.filterBlockSize(),
                        parameter.subtractedConstant(),
                        parameter.invertBlackWhiteColor());
                image.release();
                // Mat cannyimg = new Mat(image.height(), image.width(),
                // CvType.CV_8UC3);
                // Imgproc.Canny(image, cannyimg, 10, 100);
                // Imshow.show(cannyimg);

                // image.convertTo(image, -1, 1.5, 0);
                // setup camera matrix and return vectors
                // compute pose
                final Mat rvec = new Mat(3, 1, CvType.CV_64F);
                final MatOfDouble tvec = new MatOfDouble(1.0, 1.0, 1.0);
                poseProcessor.computePose(rvec, tvec, thresholdedImage);

                thresholdedImage.release();

                publishCamFrameToMarkerFrame(rvec, tvec, tfPublisherCamToMarker, connectedNode);

                // publish pose
                final QuaternionHelper q = new QuaternionHelper();

                // convert rotation vector result of solvepnp to rotation matrix
                Mat R = new Mat(3, 3, CvType.CV_32FC1);
                Calib3d.Rodrigues(rvec, R);
                // see publishers before for documentation
                final Mat tvec_map_cam = new MatOfDouble(1.0, 1.0, 1.0);
                R = R.t();
                final double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
                final double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
                final double attitudeZ = Math.asin(R.get(1, 0)[0]);
                q.setFromEuler(bankX, headingY, attitudeZ);
                Core.multiply(R, new Scalar(-1), R);
                Core.gemm(R, tvec, 1, new Mat(), 0, tvec_map_cam, 0);
                R.release();
                rvec.release();
                tvec.release();
                final org.ros.rosjava_geometry.Quaternion rotation =
                    new org.ros.rosjava_geometry.Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());
                final double x = tvec_map_cam.get(0, 0)[0];
                final double y = tvec_map_cam.get(1, 0)[0];
                final double z = tvec_map_cam.get(2, 0)[0];
                tvec_map_cam.release();

                final org.ros.rosjava_geometry.Vector3 translation =
                    new org.ros.rosjava_geometry.Vector3(x, y, z);
                final org.ros.rosjava_geometry.Transform transform_map_cam =
                    new org.ros.rosjava_geometry.Transform(translation, rotation);

                // odom to camera_rgb_optical_frame
                final GraphName sourceFrame = GraphName.of(parameter.cameraFrameName());
                final GraphName targetFrame = GraphName.of("base_link");
                org.ros.rosjava_geometry.Transform transform_cam_base = null;

                if (transformationService.canTransform(targetFrame, sourceFrame)) {
                  try {
                    transform_cam_base =
                        transformationService.lookupTransform(targetFrame, sourceFrame);
                  } catch (Exception e) {
                    log.error(ExceptionUtils.getStackTrace(e));
                    log.info(
                        "Cloud not get transformation from "
                            + parameter.cameraFrameName()
                            + " to "
                            + "base_link! "
                            + "However, will continue..");
                    // cancel this loop..no result can be computed
                    return;
                  }
                } else {
                  log.info(
                      "Cloud not get transformation from "
                          + parameter.cameraFrameName()
                          + " to "
                          + "base_link!"
                          + " However, "
                          + "will continue..");
                  // cancel this loop..no result can be computed
                  return;
                }

                // multiply results
                org.ros.rosjava_geometry.Transform current_pose =
                    org.ros.rosjava_geometry.Transform.identity();
                current_pose = current_pose.multiply(transform_map_cam);
                current_pose = current_pose.multiply(transform_cam_base);

                // check for plausibility of the pose by checking if movement
                // exceeds max speed (defined) of the robot
                if (parameter.badPoseReject()) {
                  Time current_timestamp = connectedNode.getCurrentTime();
                  // TODO Unfortunately, we do not have the tf timestamp at
                  // hand here. So we can only use the current timestamp.
                  double maxspeed = 5;
                  boolean goodpose = false;
                  // if (current_pose != null && current_timestamp != null) {
                  if (last_pose != null && last_timestamp != null) {
                    // check speed of movement between last and current pose
                    double distance = PoseCompare.distance(current_pose, last_pose);
                    double timedelta = PoseCompare.timedelta(current_timestamp, last_timestamp);
                    if ((distance / timedelta) < maxspeed) {
                      if (smoothing) {
                        double xold = last_pose.getTranslation().getX();
                        double yold = last_pose.getTranslation().getY();
                        double zold = last_pose.getTranslation().getZ();
                        double xnew = current_pose.getTranslation().getX();
                        double ynew = current_pose.getTranslation().getY();
                        double znew = current_pose.getTranslation().getZ();
                        final org.ros.rosjava_geometry.Vector3 smoothTranslation =
                            new org.ros.rosjava_geometry.Vector3(
                                (xold * 2 + xnew) / 3,
                                (yold * 2 + ynew) / 3,
                                (zold * 2 + znew) / 3);
                        current_pose =
                            new org.ros.rosjava_geometry.Transform(
                                smoothTranslation, current_pose.getRotationAndScale());
                        last_pose = current_pose;
                      }
                      last_pose = current_pose;
                      last_timestamp = current_timestamp;
                      goodpose = true;
                    } else {
                      log.info(
                          "distance " + distance + " time: " + timedelta + " --> Pose rejected");
                      log.info("current pose: " + current_pose.getTranslation().getX() + " " + current_pose.getTranslation().getY() + " " + current_pose.getTranslation().getZ());
                      log.info("last pose: " + last_pose.getTranslation().getX() + " " + last_pose.getTranslation().getY() + " " + last_pose.getTranslation().getZ());
                    }

                  } else {
                    last_pose = current_pose;
                    last_timestamp = current_timestamp;
                  }
                  // }
                  // bad pose rejection
                  if (!goodpose) {
                    return;
                  }
                }

                // set information to message
                final geometry_msgs.PoseStamped posestamped = posePublisher.newMessage();
                Pose pose = posestamped.getPose();
                Quaternion orientation = pose.getOrientation();
                Point point = pose.getPosition();

                point.setX(current_pose.getTranslation().getX());

                point.setY(current_pose.getTranslation().getY());

                point.setZ(current_pose.getTranslation().getZ());

                orientation.setW(current_pose.getRotationAndScale().getW());
                orientation.setX(current_pose.getRotationAndScale().getX());
                orientation.setY(current_pose.getRotationAndScale().getY());
                orientation.setZ(current_pose.getRotationAndScale().getZ());

                // frame_id too
                posestamped.getHeader().setFrameId("map");
                posestamped.getHeader().setStamp(connectedNode.getCurrentTime());
                posePublisher.publish(posestamped);
                mostRecentPose.set(posestamped);

              } catch (Exception e) {
                logger.info("An exception occurs.", e);
              }
            }
          }
        });
  }

  private static CameraParams getCameraInfo(
      ConnectedNode connectedNode, Parameter parameter) { // Subscribe
    // to
    // camera
    // info
    Subscriber<CameraInfo> subscriberToCameraInfo =
        connectedNode.newSubscriber(parameter.cameraInfoTopic(), CameraInfo._TYPE);
    final CameraInfoService cameraInfoService = CameraInfoService.create(subscriberToCameraInfo);
    Optional<CameraParams> cameraParamsOptional = cameraInfoService.getCameraParams();
    while (!cameraParamsOptional.isPresent()) {
      // we're not gonna do anything before getting the camera info
      try {
        TimeUnit.MILLISECONDS.sleep(100);
      } catch (InterruptedException e) {
        log.error(ExceptionUtils.getStackTrace(e));
      }
      cameraParamsOptional = cameraInfoService.getCameraParams();
    }
    return cameraParamsOptional.get();
  }

  /** @return */
  public static Log getLog() {
    return log;
  }

  private void publishCamFrameToMarkerFrame(Mat rvec, Mat tvec, Publisher<tf2_msgs.TFMessage> tfPublisherCamToMarker, ConnectedNode connectedNode) {
    QuaternionHelper q = new QuaternionHelper();

				/*
				 * http://euclideanspace.com/maths/geometry/rotations/
				 * conversions/matrixToEuler/index.htm
				 * http://stackoverflow.com/questions/12933284/rodrigues-into-
				 * eulerangles-and-vice-versa
				 *
				 * heading = atan2(-m20,m00) attitude = asin(m10) bank =
				 * atan2(-m12,m11)
				 */
    // convert output rotation vector rvec to rotation matrix R
    Mat R = new Mat(3, 3, CvType.CV_32FC1);
    Calib3d.Rodrigues(rvec, R);
    // get rotations around X,Y,Z from rotation matrix R
    double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
    double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
    double attitudeZ = Math.asin(R.get(1, 0)[0]);
    R.release();
    // convert Euler angles to quarternion
    q.setFromEuler(bankX, headingY, attitudeZ);

    // set information to message
    TFMessage tfmessage = tfPublisherCamToMarker.newMessage();
    TransformStamped posestamped = connectedNode.getTopicMessageFactory()
        .newFromType(geometry_msgs.TransformStamped._TYPE);
    Transform transform = posestamped.getTransform();

    Quaternion orientation = transform.getRotation();
    Vector3 point = transform.getTranslation();
    point.setX(tvec.get(0, 0)[0]);
    point.setY(tvec.get(1, 0)[0]);
    point.setZ(tvec.get(2, 0)[0]);

    orientation.setW(q.getW());
    orientation.setX(q.getX());
    orientation.setY(q.getY());
    orientation.setZ(q.getZ());
    posestamped.getHeader().setFrameId(parameter.cameraFrameName());
    posestamped.setChildFrameId(parameter.markerFrameName());
    posestamped.getHeader().setStamp(connectedNode.getCurrentTime());
    // frame_id too
    tfmessage.getTransforms().add(posestamped);
    tfPublisherCamToMarker.publish(tfmessage);
  }

  @Override
  public Optional<PoseStamped> getMostRecentPose() {
    final PoseStamped poseStamped = mostRecentPose.get();
    if (poseStamped == null) {
      return Optional.absent();
    } else {
      return Optional.of(poseStamped);
    }
  }
}

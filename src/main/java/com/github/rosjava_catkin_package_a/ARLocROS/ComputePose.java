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

package com.github.rosjava_catkin_package_a.ARLocROS;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;

import jp.nyatla.nyartoolkit.core.NyARCode;
import jp.nyatla.nyartoolkit.core.NyARException;
import jp.nyatla.nyartoolkit.core.param.INyARCameraDistortionFactor;
import jp.nyatla.nyartoolkit.core.param.NyARCameraDistortionFactorV2;
import jp.nyatla.nyartoolkit.core.param.NyARParam;
import jp.nyatla.nyartoolkit.core.param.NyARPerspectiveProjectionMatrix;
import jp.nyatla.nyartoolkit.core.raster.rgb.INyARRgbRaster;
import jp.nyatla.nyartoolkit.core.raster.rgb.NyARRgbRaster;
import jp.nyatla.nyartoolkit.core.types.NyARBufferType;
import jp.nyatla.nyartoolkit.core.types.NyARIntPoint2d;
import jp.nyatla.nyartoolkit.core.types.NyARIntSize;
import jp.nyatla.nyartoolkit.markersystem.INyARMarkerSystemConfig;
import jp.nyatla.nyartoolkit.markersystem.NyARMarkerSystem;
import jp.nyatla.nyartoolkit.markersystem.NyARMarkerSystemConfig;
import jp.nyatla.nyartoolkit.markersystem.NyARSensor;

public class ComputePose {

	public static boolean computePose(Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs, Mat image2,
			Size size, String pattern_dir, String mARKER_CONFIG_FILE) throws NyARException, FileNotFoundException {
		// read marker patterns from config and init config
		List<String> markerPatterns = MarkerConfig.readFromConfig(mARKER_CONFIG_FILE);
		//
		// List<NyARCode> codeList = new ArrayList<>();
		// create hasmap of correspondences between id and file/pattern name
		Map<Integer, String> patternmap = new HashMap<>();
		//
		NyARIntSize i_screen_size = new NyARIntSize((int) size.width, (int) size.height);
		NyARPerspectiveProjectionMatrix i_projection_mat = new NyARPerspectiveProjectionMatrix();
		INyARCameraDistortionFactor i_dist_factor = new NyARCameraDistortionFactorV2();
		//
		NyARParam i_param = new NyARParam(i_screen_size, i_projection_mat, i_dist_factor);
		//
		INyARRgbRaster imageRaster = NyARImageHelper.createFromMat(image2);

		INyARMarkerSystemConfig i_config = new NyARMarkerSystemConfig(i_param);
		NyARMarkerSystem markerSystemState = new NyARMarkerSystem(i_config);
		int[] ids = new int[markerPatterns.size()];
		for (int i = 0; i < markerPatterns.size(); i++) {
			NyARCode code = NyARCode.createFromARPattFile(new FileInputStream(pattern_dir + markerPatterns.get(i)), 16,
					16);
			// codeList.add(code);
			//System.out.print(MarkerConfig.getMarkerSize());
			ids[i] = markerSystemState.addARMarker(code, 25, MarkerConfig.getMarkerSize());
			patternmap.put(ids[i], markerPatterns.get(i));
		}
		// System.out.println();
		//
		NyARSensor cameraSensorWrapper = new NyARSensor(i_screen_size);
		cameraSensorWrapper.update(imageRaster);
		markerSystemState.update(cameraSensorWrapper);
		// init 3d point list
		List<Point3> points3dlist = new ArrayList<>();
		// System.out.print("Confidence: ");
		List<Point> points2dlist = new ArrayList<Point>();
		for (int i = 0; i < ids.length; i++) {
			if (markerSystemState.isExistMarker(ids[i])) {
				// read and add 2d points
				NyARIntPoint2d[] vertex2d = markerSystemState.getMarkerVertex2D(ids[i]);
				//System.out.print(patternmap.get(ids[i]) + " ");

				// Point p = new Point(vertex2d[2].x, vertex2d[2].y);
				// points2dlist.add(p);
				// p = new Point(vertex2d[3].x, vertex2d[3].y);
				// points2dlist.add(p);
				// p = new Point(vertex2d[0].x, vertex2d[0].y);
				// points2dlist.add(p);
				// p = new Point(vertex2d[1].x, vertex2d[1].y);
				// points2dlist.add(p);
				Point p = new Point(vertex2d[0].x, vertex2d[0].y);
				points2dlist.add(p);
				p = new Point(vertex2d[1].x, vertex2d[2].y);
				points2dlist.add(p);
				p = new Point(vertex2d[2].x, vertex2d[2].y);
				points2dlist.add(p);
				p = new Point(vertex2d[3].x, vertex2d[3].y);
				points2dlist.add(p);

				MatOfPoint mop = new MatOfPoint();
				mop.fromList(points2dlist);
				List<MatOfPoint> pts = new ArrayList<MatOfPoint>();
				pts.add(mop);
				// read and add corresponding 3d points
				points3dlist.addAll(MarkerConfig.create3dpointlist(patternmap.get(ids[i])));
			}
			// Imgproc.drawContours(image2, pts, -1, new Scalar(0, 0, 255));
		}
		//System.out.println();
//		for (int i = 0; i < points2dlist.size(); i++) {
//			System.out.println(points2dlist.get(i) + "-->" + points3dlist.get(i));
//		}
		// System.out.println();
		//Imshow.show(image2);
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		// List<Point3> points3dlist = MarkerConfig.create3dpointlist();
		objectPoints.fromList(points3dlist);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		imagePoints.fromList(points2dlist);

		// for (int i = 0; i < points2dlist.size(); i++) {
		// System.out.println(points2dlist.get(i) + "-->" +
		// points3dlist.get(i));
		// }

		if (points2dlist.size() == 0)
			return false;

		// Mat inliers = new Mat();
		// Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix,
		// distCoeffs, rvec, tvec, false, 500, 2, 16,
		// inliers, Calib3d.CV_EPNP);
		Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

		// System.out.println(inliers.dump());
		// Mat R = new Mat(3, 3, CvType.CV_32FC1);
		// Calib3d.Rodrigues(rvec, R);
		// R = R.t();
		// Calib3d.Rodrigues(R, rvec);
		// //
		// Core.multiply(R, new Scalar(-1), R);
		// // Calib3d.Rodrigues(R, rvec);
		//
		// //
		// Core.gemm(R, tvec, 1, new Mat(), 0, tvec, 0);
		// System.out.println(tvec.dump() + " " + rvec.dump());
		return true;

	}

}

class NyARImageHelper extends NyARRgbRaster {

	public static INyARRgbRaster createFromMat(Mat image) {
		BufferedImage bimg;
		if (image != null) {
			int cols = image.cols();
			int rows = image.rows();
			int elemSize = (int) image.elemSize();
			byte[] data = new byte[cols * rows * elemSize];
			int type;
			image.get(0, 0, data);
			// we only support RGB
			type = BufferedImage.TYPE_3BYTE_BGR;
			// bgr to rgb
			byte b;
			for (int i = 0; i < data.length; i = i + 3) {
				b = data[i];
				data[i] = data[i + 2];
				data[i + 2] = b;
			}

			bimg = new BufferedImage(cols, rows, type);

			bimg.getRaster().setDataElements(0, 0, cols, rows, data);
		} else {
			bimg = null;
		}

		NyARImageHelper ra = null;

		int raster_type = NyARBufferType.BYTE1D_B8G8R8_24;
		try {
			ra = new NyARImageHelper(bimg.getWidth(), bimg.getHeight(), raster_type, false);
			ra._buf = ((DataBufferByte) (bimg.getRaster().getDataBuffer())).getData();
			ra._rgb_pixel_driver.switchRaster(ra);
		} catch (NyARException e) {
			e.printStackTrace();
		}

		return ra;

	}

	public NyARImageHelper(int i_width, int i_height, int i_raster_type, boolean i_is_alloc) throws NyARException {

		super(i_width, i_height, i_raster_type, i_is_alloc);
	}

}

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

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import sensor_msgs.Image;

import java.util.Arrays;

public class Utils {

  static public Mat matFromImage(final Image source) throws Exception {
    byte[] imageInBytes = source.getData().array();
    imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(),
        imageInBytes.length);
    Mat cvImage = new Mat(source.getHeight(), source.getWidth(), CvType.CV_8UC3);
    cvImage.put(0, 0, imageInBytes);
    return cvImage;
  }

  static public Mat tresholdContrastBlackWhite(Mat srcImage, int filterBlockSize, double subtractedConstant,
      boolean invertBlackWhiteColor) {

    final Mat transformMat = new Mat(1, 3, CvType.CV_32F);
    final int row = 0;
    final int col = 0;
    transformMat.put(row, col, 0.33, 0.33, 0.34);

    final Mat grayImage = new Mat(srcImage.height(), srcImage.width(), CvType.CV_8UC1);
    Core.transform(srcImage, grayImage, transformMat);

    Mat thresholdedImage = new Mat(grayImage.height(), grayImage.width(), CvType.CV_8UC1);
    Imgproc.adaptiveThreshold(grayImage, thresholdedImage, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, filterBlockSize, subtractedConstant);
    grayImage.release();

    if (invertBlackWhiteColor) {
      final Mat invertedImage = new Mat(thresholdedImage.height(), thresholdedImage.width(), CvType.CV_8UC1);
      Core.bitwise_not(thresholdedImage, invertedImage);
      thresholdedImage.release();
      thresholdedImage = invertedImage;
    }

    final Mat coloredImage = new Mat(thresholdedImage.height(), thresholdedImage.width(), CvType.CV_8UC3);
    Imgproc.cvtColor(thresholdedImage, coloredImage, Imgproc.COLOR_GRAY2RGB);
    thresholdedImage.release();
    return coloredImage;
  }
}

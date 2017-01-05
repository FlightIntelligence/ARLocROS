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

  static public void tresholdContrastBlackWhite(Mat image2, int filterBlockSize, double subtractedConstant,
      boolean invertBlackWhiteColor) {
//		int width = image2.width();
//		int height = image2.height();
//		for (int i = 0; i < width; i++)
//			for (int j = 0; j < height; j++) {
//				double[] rgb = image2.get(j, i);
//				double[] rgbnew = new double[rgb.length];
//				if (rgb[0] + rgb[1] + rgb[2] < d)
//					rgbnew[0] = rgbnew[1] = rgbnew[2] = 0.0;
//				else
//					rgbnew[0] = rgbnew[1] = rgbnew[2] = 255.0;
//				image2.put(j, i, rgbnew);
//			}

    final Mat transformMat = new Mat(1, 3, CvType.CV_32F);
    final int row = 0;
    final int col = 0;
    transformMat.put(row, col, 0.33, 0.33, 0.34);
    image2.convertTo(image2, CvType.CV_32F);
    Core.transform(image2, image2, transformMat);
    Imgproc.adaptiveThreshold(image2, image2, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, filterBlockSize, subtractedConstant);
    if (invertBlackWhiteColor) {
      Core.bitwise_not(image2, image2);
    }
    Imgproc.cvtColor(image2, image2, Imgproc.COLOR_GRAY2RGB);
  }
}

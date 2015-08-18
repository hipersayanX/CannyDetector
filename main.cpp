/* CannyDetector, Implementation of Canny edge detector in Qt/C++.
 * Copyright (C) 2015  Gonzalo Exequiel Pedone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Email   : hipersayan DOT x AT gmail DOT com
 * Web-Site: http://github.com/hipersayanX/CannyDetector
 */

#include <iostream>
#include <cmath>
#include <QCoreApplication>
#include <QImage>

inline void sobel(const QImage &image,
                  QVector<int> &gradient,
                  QVector<int> &direction)
{
    int size = image.width() * image.height();
    gradient.resize(size);
    direction.resize(size);

    for (int y = 0; y < image.height(); y++) {
        size_t yOffset = y * image.width();
        const quint8 *grayLine = image.constBits() + yOffset;

        const quint8 *grayLine_m1 = y < 1? grayLine: grayLine - image.width();
        const quint8 *grayLine_p1 = y >= image.height() - 1? grayLine: grayLine + image.width();

        int *gradientLine = gradient.data() + yOffset;
        int *directionLine = direction.data() + yOffset;

        for (int x = 0; x < image.width(); x++) {
            int x_m1 = x < 1? x: x - 1;
            int x_p1 = x >= image.width() - 1? x: x + 1;

            int gradX = grayLine_m1[x_p1]
                      + 2 * grayLine[x_p1]
                      + grayLine_p1[x_p1]
                      - grayLine_m1[x_m1]
                      - 2 * grayLine[x_m1]
                      - grayLine_p1[x_m1];

            int gradY = grayLine_m1[x_m1]
                      + 2 * grayLine_m1[x]
                      + grayLine_m1[x_p1]
                      - grayLine_p1[x_m1]
                      - 2 * grayLine_p1[x]
                      - grayLine_p1[x_p1];

            gradientLine[x] = qAbs(gradX) + qAbs(gradY);

            /* Gradient directions are classified in 4 possible cases
             *
             * dir 0
             *
             * x x x
             * - - -
             * x x x
             *
             * dir 1
             *
             * x x /
             * x / x
             * / x x
             *
             * dir 2
             *
             * \ x x
             * x \ x
             * x x \
             *
             * dir 3
             *
             * x | x
             * x | x
             * x | x
             */
            if (gradX == 0 && gradY == 0)
                directionLine[x] = 0;
            else if (gradX == 0)
                directionLine[x] = 3;
            else {
                qreal a = 180. * atan(qreal(gradY) / gradX) / M_PI;

                if (a >= -22.5 && a < 22.5)
                    directionLine[x] = 0;
                else if (a >= 22.5 && a < 67.5)
                    directionLine[x] = 1;
                else if (a >= -67.5 && a < -22.5)
                    directionLine[x] = 2;
                else
                    directionLine[x] = 3;
            }
        }
    }
}

inline QVector<int> thinning(int width, int height,
                             const QVector<int> &gradient,
                             const QVector<int> &direction)
{
    QVector<int> thinned(gradient.size());

    for (int y = 0; y < height; y++) {
        int yOffset = y * width;
        const int *gradientLine = gradient.constData() + yOffset;
        const int *gradientLine_m1 = y < 1? gradientLine: gradientLine - width;
        const int *gradientLine_p1 = y >= height - 1? gradientLine: gradientLine + width;
        const int *directionLine = direction.constData() + yOffset;
        int *thinnedLine = thinned.data() + yOffset;

        for (int x = 0; x < width; x++) {
            int x_m1 = x < 1? 0: x - 1;
            int x_p1 = x >= width - 1? x: x + 1;

            int direction = directionLine[x];
            int pixel = 0;

            if (direction == 0) {
                /* x x x
                 * - - -
                 * x x x
                 */
                if (gradientLine[x] < gradientLine[x_m1]
                    || gradientLine[x] < gradientLine[x_p1])
                    pixel = 0;
                else
                    pixel = gradientLine[x];
            } else if (direction == 1) {
                /* x x /
                 * x / x
                 * / x x
                 */
                if (gradientLine[x] < gradientLine_m1[x_p1]
                    || gradientLine[x] < gradientLine_p1[x_m1])
                    pixel = 0;
                else
                    pixel = gradientLine[x];
            } else if (direction == 2) {
                /* \ x x
                 * x \ x
                 * x x \
                 */
                if (gradientLine[x] < gradientLine_m1[x_m1]
                    || gradientLine[x] < gradientLine_p1[x_p1])
                    pixel = 0;
                else
                    pixel = gradientLine[x];
            } else {
                /* x | x
                 * x | x
                 * x | x
                 */
                if (gradientLine[x] < gradientLine_m1[x]
                    || gradientLine[x] < gradientLine_p1[x])
                    pixel = 0;
                else
                    pixel = gradientLine[x];
            }

            thinnedLine[x] = pixel;
        }
    }

    return thinned;
}

inline QVector<int> threshold(int thLow, int thHi,
                              const QVector<int> &image)
{
    QVector<int> thresholded(image.size());

    for (int i = 0; i < image.size(); i++)
        thresholded[i] = image[i] <= thLow? 0:
                         image[i] >= thHi? 255:
                                           127;

    return thresholded;
}

void trace(int width, int height, QVector<int> &image, int x, int y)
{
    int yOffset = y * width;
    int *cannyLine = image.data() + yOffset;

    if (cannyLine[x] != 255)
        return;

    for (int j = -1; j < 2; j++) {
        int nextY = y + j;

        if (nextY < 0 || nextY >= height)
            continue;

        int *cannyLineNext = cannyLine + j * width;

        for (int i = -1; i < 2; i++) {
            int nextX = x + i;

            if (i == j || nextX < 0 || nextX >= width)
                continue;

            if (cannyLineNext[nextX] == 127) {
                // Mark pixel as white.
                cannyLineNext[nextX] = 255;

                // Trace neighbors.
                trace(width, height, image, nextX, nextY);
            }
        }
    }
}

QVector<int> hysteresis(int width, int height,
                        const QVector<int> &image)
{
    QVector<int> canny(image);

    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            trace(width, height, canny, x, y);

    // Remaining gray pixels becomes black.
    for (int i = 0; i < canny.size(); i++)
        if (canny[i] == 127)
            canny[i] = 0;

    return canny;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Q_UNUSED(a)

    QImage inImage("lena.png");
    inImage = inImage.convertToFormat(QImage::Format_Grayscale8);
    QImage outImage(inImage.size(), inImage.format());

    QVector<int> gradient;
    QVector<int> direction;
    sobel(inImage, gradient, direction);
    QVector<int> thinned = thinning(inImage.width(), inImage.height(),
                                   gradient, direction);
    QVector<int> thresholded = threshold(75, 150, thinned);
    QVector<int> canny = hysteresis(inImage.width(), inImage.height(),
                                    thresholded);

    const int *iImg = canny.constData();
    quint8 *oImg = outImage.bits();

    int size = inImage.width() * inImage.height();

    for (int i = 0; i < size; i++)
        oImg[i] = qBound(0, iImg[i], 255);

    outImage.save("canny.png");

    return EXIT_SUCCESS;
}

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Drawing;
using Emgu;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.ComponentModel;
using ImageProccesingWPF;
using ChartDirector;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;


namespace MainCode
{

    /// <summary>
    /// Interaction logic for ImageControlPage.xaml
    /// </summary>
    public partial class ImageControlPage : Page, INotifyPropertyChanged
    {
        const int SERVO_X_MIN = 80;
        const int SERVO_X_MAX = 120;
        const int SERVO_Y_MIN = 35;
        const int SERVO_Y_MAX = 75;
 

        const int SERVO_X_HOME = 100;
        const int SERVO_Y_HOME = 57;

        const double SampleTime = 0.01;
        #region Serial UART
        //Richtextbox
        FlowDocument mcFlowDoc = new FlowDocument();
        Paragraph para = new Paragraph();
        //Serial 
        SerialPort serial = new SerialPort();
        #endregion
        Stopwatch tStart;
        //Stopwatch sw;

        public ImageControlPage()
        {
            InitializeComponent();
            if (_capture != null)
            {
                _capture.ImageGrabbed += ProcessFrame;
                _capture.Start();
            }
            tBar_HLo.Value = 34;
            tBar_HHi.Value = 155;
            tBar_SLo.Value = 0;
            tBar_SHi.Value = 255;
            tBar_VLo.Value = 157;
            tBar_VHi.Value = 255;

            tBar_PX.Value = 100;
            tBar_IX.Value = 0;
            tBar_DX.Value = 100;
            tBar_PY.Value = 100;
            tBar_IY.Value = 0;
            tBar_DY.Value = 100;

            Status.Badge = "No Connect";
        }


        VideoCapture _capture = new VideoCapture(0);

        Mat frame = new Mat();
        Mat frame_ROI = new Mat();
        Mat frameRGB = new Mat();
        Mat frameHSV = new Mat();
        Mat frameGray = new Mat();

        List<Mat> imgShowList = new List<Mat>();

        private int hueLow;
        private int hueHigh;
        private int satLow;
        private int satHigh;
        private int valLow;
        private int valHigh;

        private int valPX;
        private int valIX;
        private int valDX;
        private int valPY;
        private int valIY;
        private int valDY;

        private int posX = 0;
        private int posY = 0;
        private int alpha = 0;
        private int beta = 0;
        private int setpointx = 0;
        private int setpointy = 0;

        #region Initialize PID
        private DispatcherTimer PID_calculate = new DispatcherTimer(DispatcherPriority.Render);
        #endregion
        #region Initialize Variable of Plot
        // The data arrays that store the realtime data. The data arrays are updated in realtime. 
        // In this demo, we store at most 10000 values. 
        private const int sampleSize = 10000;
        private double[] timeStamps = new double[sampleSize];
        private double[] dataSeriesA = new double[sampleSize];
        private double[] dataSeriesB = new double[sampleSize];
        private double[] dataSeriesC = new double[sampleSize];
        private double[] dataSeriesD = new double[sampleSize];

        // The index of the array position to which new data values are added.
        private int currentIndex = 0;

        // The full range is initialized to 60 seconds of data. It can be extended when more data
        // are available.
        private int initialFullRange = 30;

        // The maximum zoom in is 10 seconds.
        private int zoomInLimit = 10;

        // In this demo, we use a data generator driven by a timer to generate realtime data. The
        // nextDataTime is an internal variable used by the data generator to keep track of which
        // values to generate next.
        private DispatcherTimer dataRateTimer = new DispatcherTimer(DispatcherPriority.Render);
        private DateTime nextDataTime = new DateTime(DateTime.Now.Ticks / 10000000 * 10000000);
        // Timer used to updated the chart
        private DispatcherTimer chartUpdateTimer = new DispatcherTimer(DispatcherPriority.Render);

        // The first and last chart time
        DateTime firstChartTime = DateTime.MinValue;
        double chartTimeLimit;

        // The position of the track lines
        List<int> trackLinePos = new List<int>();

        // Keep track of the mouse for dragging the track lines
        int nearestTrackLine = -1;
        int nearestDistance;
        #endregion


        public event PropertyChangedEventHandler PropertyChanged;
        private void Page_Loaded(object sender, RoutedEventArgs e)
        {
            RadioSetPoint.IsChecked = true;
            #region Initialize multithread of PID
            PID_calculate.Interval = new TimeSpan(0, 0, 0, 0, (int)(SampleTime*1000));
            PID_calculate.Tick += PID_calculate_Tick;
            PID_calculate.Start();
            #endregion
            #region Initialize multithread of Plot
            initChartViewer(WPFChartViewer1);
            // Data generation rate = 250ms
            dataRateTimer.Interval = new TimeSpan(0, 0, 0, 0, 50);
            dataRateTimer.Tick += dataRateTimer_Tick;
            // Chart update rate, which can be different from the data generation rate.
            chartUpdateTimer.Interval = new TimeSpan(0, 0, 0, 0, 50);
            chartUpdateTimer.Tick += chartUpdateTimer_Tick;

            // Initialize data buffer to no data.
            for (int i = 0; i < timeStamps.Length; ++i)
                timeStamps[i] = Chart.NoValue;

            // Now can start the timers for data collection and chart update
            dataRateTimer.Start();
            chartUpdateTimer.Start();
            #endregion
            Comm_Port_Names.ItemsSource= SerialPort.GetPortNames();
        }
        #region Function of calculate PID
        double lastErrorX = 0, lastErrorY = 0;
        double dArea = 0;
        double PIDx, PIDy;
        double errorX = 0, errorY = 0;
        double sumErrorX = 0, sumErrorY = 0;
        double SetX, SetY;
        int nFlag = 0;
        private void PID_calculate_Tick(object sender, EventArgs e)
        {
            if (dArea > 5000)
            {
                
                double KP_X = (double)PX / 1000;
                double KI_X = (double)IX / 1000;
                double KD_X = (double)DX / 100;

                double KP_Y = (double)PY / 1000;
                double KI_Y = (double)IY / 1000;
                double KD_Y = (double)DY / 100;

                if (Check == 0)
                {
                    SetX = SetPointX;
                    SetY = SetPointY;
                }
                else if (Check == 1)
                {
                    if ((tStart.ElapsedMilliseconds >= 4000))
                    {
                        switch (nFlag)
                        {
                            case 0:
                                SetX = 0;
                                SetY = 70;
                                nFlag++;
                                break;
                            case 1:
                                SetX = -80;
                                SetY = -70;
                                nFlag++;
                                break;
                            case 2:
                                SetX = 80;
                                SetY = -70;
                                nFlag = 0;
                                break;
                        }
                        tStart = Stopwatch.StartNew();
                    }
                }
                else if (Check == 2)
                {
                    if ((tStart.ElapsedMilliseconds >= 4000))
                    {
                        switch (nFlag)
                        {
                            case 0:
                                SetX = 70;
                                SetY = 70;
                                nFlag++;
                                break;
                            case 1:
                                SetX = -70;
                                SetY = 70;
                                nFlag++;
                                break;
                            case 2:
                                SetX = -70;
                                SetY = -70;
                                nFlag++;
                                break;
                            case 3:
                                SetX = 70;
                                SetY = -70;
                                nFlag = 0;
                                break;
                        }
                        tStart = Stopwatch.StartNew();
                    }
                }
                else if (Check == 3)
                {
                    if ((tStart.ElapsedMilliseconds >= 10))
                    {


                        SetY = 40 * Math.Sin(nFlag * 3.14159 / 180);
                        SetX = 40 * Math.Cos(nFlag * 3.14159 / 180);
                        nFlag += 12;

                        tStart = Stopwatch.StartNew();
                    }
                }
                errorX = PosX - SetX;
                //generalized PID formula
                PIDx = KP_X * errorX + KD_X * (errorX - lastErrorX) + KI_X * (sumErrorX);
                lastErrorX = errorX;
                sumErrorX += errorX;

                errorY = PosY - SetY;
                PIDy = KP_Y * errorY + KD_Y * (errorY - lastErrorY) + KI_Y * (sumErrorY);
                lastErrorY = errorY;
                sumErrorY += errorY;
                if (sumErrorX > 300)
                {
                    sumErrorX = 300;
                }
                else if (sumErrorX < -300)
                {
                    sumErrorX = -300;
                }

                if (sumErrorY > 300)
                {
                    sumErrorY = 300;
                }
                else if (sumErrorY < -300)
                {
                    sumErrorY = -300;
                }

                int OutputX = (int)(PIDx + HomeX);
                if (OutputX > SERVO_X_MAX)
                    OutputX = SERVO_X_MAX;
                if (OutputX < SERVO_X_MIN)
                    OutputX = SERVO_X_MIN;

                int OutputY = (int)(PIDy + HomeY);
                if (OutputY > SERVO_Y_MAX)
                    OutputY = SERVO_Y_MAX;
                if (OutputY < SERVO_Y_MIN)
                    OutputY = SERVO_Y_MIN;
                Alpha = (int)PIDx;
                Beta = (int)PIDy;
                string str = OutputX.ToString() + " " + OutputY.ToString() + "\n";
                SerialCmdSend(str);
                //double P = (double)PX / 1000;
                //double I = (double)IX;
                //double D = (double)DX / 100;
                //double  errorX = PosX - 0;
                //double errorY = (PosY - 0);

                // //generalized PID formula
                // double PIDx = P * errorX + D * (errorX - lastErrorX) + I * (sumErrorX);
                // double PIDy = P * errorY + D * (errorY - lastErrorY) + I * (sumErrorY);

                // lastErrorX = errorX;
                // lastErrorY = errorY;

                // sumErrorX += errorX;
                // sumErrorY += errorY;

                // //scale the sum for the integral term

                // PIDx = PIDx + SERVO_X_HOME;
                // PIDy = PIDy + SERVO_Y_HOME;
                // if (PIDx > SERVO_X_MAX)
                //     PIDx = SERVO_X_MAX;
                // if (PIDx < SERVO_X_MIN)
                //     PIDx = SERVO_X_MIN;

                // if (PIDy > SERVO_Y_MAX)
                //     PIDy = SERVO_Y_MAX;
                // if (PIDy < SERVO_Y_MIN)
                //     PIDy = SERVO_Y_MIN;
                // string str = ((int)PIDx).ToString() + " " + ((int)PIDy).ToString() + "\n";
                // SerialCmdSend(str);

            }

        }
        #endregion
            #region Function of Plot
        private void initChartViewer(WPFChartViewer viewer)
        {
            viewer.MouseWheelZoomRatio = 1.1;
        }

        private void dataRateTimer_Tick(object sender, EventArgs e)
        {
            do
            {
                //
                // In this demo, we use some formulas to generate new values. In real applications,
                // it may be replaced by some data acquisition code.
                //
                double p = nextDataTime.Ticks / 10000000.0 * 4;
                //Add Data
                double dataA = PosX;
                double dataB = PosY;
                double dataC = SetX;
                double dataD = SetY;


                // In this demo, if the data arrays are full, the oldest 5% of data are discarded.
                if (currentIndex >= timeStamps.Length)
                {
                    currentIndex = sampleSize * 95 / 100 - 1;

                    for (int i = 0; i < currentIndex; ++i)
                    {
                        int srcIndex = i + sampleSize - currentIndex;
                        timeStamps[i] = timeStamps[srcIndex];
                        dataSeriesA[i] = dataSeriesA[srcIndex];
                        dataSeriesB[i] = dataSeriesB[srcIndex];
                        dataSeriesC[i] = dataSeriesC[srcIndex];
                        dataSeriesD[i] = dataSeriesD[srcIndex];
                    }
                }
                // Remember the first timestamps to compute the elapsed time
                if (firstChartTime == DateTime.MinValue)
                    firstChartTime = nextDataTime;

                // Store the new values in the current index position, and increment the index.
                timeStamps[currentIndex] = nextDataTime.Subtract(firstChartTime).TotalSeconds;
                dataSeriesA[currentIndex] = dataA;
                dataSeriesB[currentIndex] = dataB;
                dataSeriesC[currentIndex] = dataC;
                dataSeriesD[currentIndex] = dataD;
                ++currentIndex;

                nextDataTime = nextDataTime.AddMilliseconds(dataRateTimer.Interval.TotalMilliseconds);
            } while (nextDataTime < DateTime.Now);
        }
        private void chartUpdateTimer_Tick(object sender, EventArgs e)
        {
            var viewer = WPFChartViewer1;

            if (currentIndex >= 0)
            {
                //
                // As we added more data, we may need to update the full range. 
                //

                double startDate = timeStamps[0];
                double endDate = timeStamps[currentIndex - 1];

                // Use the initialFullRange if this is sufficient.
                double duration = endDate - startDate;
                if (duration < initialFullRange)
                    endDate = startDate + initialFullRange;

                // Update the full range to reflect the actual duration of the data. In this case, 
                // if the view port is viewing the latest data, we will scroll the view port as new
                // data are added. If the view port is viewing historical data, we would keep the 
                // axis scale unchanged to keep the chart stable.
                int updateType = Chart.ScrollWithMax;
                if (viewer.ViewPortLeft + viewer.ViewPortWidth < 0.999)
                    updateType = Chart.KeepVisibleRange;
                bool axisScaleHasChanged = viewer.updateFullRangeH("x", startDate, endDate, updateType);

                // Set the zoom in limit as a ratio to the full range
                viewer.ZoomInWidthLimit = zoomInLimit / (viewer.getValueAtViewPort("x", 1) -
                    viewer.getValueAtViewPort("x", 0));

                // Trigger the viewPortChanged event to update the display if the axis scale has 
                // changed or if new data are added to the existing axis scale.
                if (axisScaleHasChanged || (duration < initialFullRange))
                    viewer.updateViewPort(true, false);
            }
        }

        private void WPFChartViewer1_ViewPortChanged(object sender, ChartDirector.WPFViewPortEventArgs e)
        {
            var viewer = sender as WPFChartViewer;

            // In addition to updating the chart, we may also need to update other controls that
            // changes based on the view port.
            updateControls(viewer);

            // Update the chart if necessary
            if (e.NeedUpdateChart)
                drawChart(viewer);
        }

        private void updateControls(WPFChartViewer viewer)
        {
            // Update the scroll bar to reflect the view port position and width.
            hScrollBar1.IsEnabled = viewer.ViewPortWidth < 1;
            hScrollBar1.LargeChange = viewer.ViewPortWidth * (hScrollBar1.Maximum - hScrollBar1.Minimum);
            hScrollBar1.SmallChange = hScrollBar1.LargeChange * 0.1;
            hScrollBar1.ViewportSize = viewer.ViewPortWidth / Math.Max(1E-10, 1 - viewer.ViewPortWidth)
                * (hScrollBar1.Maximum - hScrollBar1.Minimum);
            hScrollBar1.Value = viewer.ViewPortLeft / Math.Max(1E-10, 1 - viewer.ViewPortWidth)
                * (hScrollBar1.Maximum - hScrollBar1.Minimum) + hScrollBar1.Minimum;
        }

        private void drawChart(WPFChartViewer viewer)
        {
            // Get the start date and end date that are visible on the chart.
            double viewPortStartDate = viewer.getValueAtViewPort("x", viewer.ViewPortLeft);
            double viewPortEndDate = viewer.getValueAtViewPort("x", viewer.ViewPortLeft +
                viewer.ViewPortWidth);

            // Extract the part of the data arrays that are visible.
            double[] viewPortTimeStamps = null;
            double[] viewPortDataSeriesA = null;
            double[] viewPortDataSeriesB = null;
            double[] viewPortDAtaSeriesC = null;
            double[] viewPortDAtaSeriesD = null;
            if (currentIndex > 0)
            {
                // Get the array indexes that corresponds to the visible start and end dates
                int startIndex = (int)Math.Floor(Chart.bSearch2(timeStamps, 0, currentIndex, viewPortStartDate));
                int endIndex = (int)Math.Ceiling(Chart.bSearch2(timeStamps, 0, currentIndex, viewPortEndDate));
                int noOfPoints = endIndex - startIndex + 1;

                // Extract the visible data
                viewPortTimeStamps = (double[])Chart.arraySlice(timeStamps, startIndex, noOfPoints);
                viewPortDataSeriesA = (double[])Chart.arraySlice(dataSeriesA, startIndex, noOfPoints);
                viewPortDataSeriesB = (double[])Chart.arraySlice(dataSeriesB, startIndex, noOfPoints);
                viewPortDAtaSeriesC = (double[])Chart.arraySlice(dataSeriesC, startIndex, noOfPoints);
                viewPortDAtaSeriesD = (double[])Chart.arraySlice(dataSeriesD, startIndex, noOfPoints);
                chartTimeLimit = timeStamps[currentIndex - 1];
            }

            //
            // At this stage, we have extracted the visible data. We can use those data to plot the chart.
            //

            //================================================================================
            // Configure overall chart appearance.
            //================================================================================

            // Create an XYChart object of size 640 x 350 pixels
            XYChart c = new XYChart(640, 350);

            // Set the position, size and colors of the plot area
            c.setPlotArea(23, 33, c.getWidth() - 41, c.getHeight() - 62, c.linearGradientColor(0, 33, 0,
                c.getHeight() - 53, 0xf0f6ff, 0xa0c0ff), -1, Chart.Transparent, 0xffffff, 0xffffff);

            // As the data can lie outside the plotarea in a zoomed chart, we need enable clipping.
            c.setClipping();

            // Add a title to the chart using 18 pts Arial font
            c.addTitle("Position ball", "Arial", 18);

            // Add a legend box at (60, 28) using horizontal layout. Use 8pts Arial Bold as font. Set the
            // background and border color to Transparent and use line style legend key.
            LegendBox b = c.addLegend(60, 28, false, "Arial Bold", 10);
            b.setBackground(Chart.Transparent);
            b.setLineStyleKey();

            // Set the x and y axis stems to transparent and the label font to 10pt Arial
            c.xAxis().setColors(Chart.Transparent);
            c.yAxis().setColors(Chart.Transparent);
            c.xAxis().setLabelStyle("Arial", 10);
            c.yAxis().setLabelStyle("Arial Bold", 10, 0x336699);

            // Set the y-axis tick length to 0 to disable the tick and put the labels closer to the axis.
            c.yAxis().setLabelGap(-1);
            c.yAxis().setLabelAlignment(1);
            c.yAxis().setTickLength(0);
            c.yAxis().setMargin(20);

            // Add axis title using 12pts Arial Bold Italic font
            c.yAxis().setTitle("Position Ball (mm)", "Arial Bold", 12);

            // Configure the x-axis tick length to 1 to put the labels closer to the axis.
            c.xAxis().setTickLength(1);

            //================================================================================
            // Add data to chart
            //================================================================================

            //
            // In this example, we represent the data by lines. You may modify the code below to use other
            // representations (areas, scatter plot, etc).
            //

            // Add a line layer for the lines, using a line width of 2 pixels
            LineLayer layer = c.addLineLayer2();
            layer.setLineWidth(2);
            layer.setFastLineMode();

            // Now we add the 3 data series to a line layer, using the color red (ff0000), green (00cc00)
            // and blue (0000ff)
            layer.setXData(viewPortTimeStamps);
            layer.addDataSet(viewPortDataSeriesA, 0x00cc00, "Position X");
            layer.addDataSet(viewPortDataSeriesB, 0x0000ff, "Position Y");
            layer.addDataSet(viewPortDAtaSeriesC, 0xFF0000, "Set X");
            layer.addDataSet(viewPortDAtaSeriesD, 0xFFFF00, "Set Y");
            //================================================================================
            // Configure axis scale and labelling
            //================================================================================

            if (currentIndex > 0)
                c.xAxis().setDateScale(viewPortStartDate, viewPortEndDate);

            // For the automatic axis labels, set the minimum spacing to 75/30 pixels for the x/y axis.
            c.xAxis().setTickDensity(75);
            c.yAxis().setTickDensity(30);

            // We use "hh:nn:ss" as the axis label format.
            c.xAxis().setLabelFormat("{value|nn:ss}");

            // We make sure the tick increment must be at least 1 second.
            c.xAxis().setMinTickInc(1);

            //================================================================================
            // Output the chart
            //================================================================================

            // We need to update the track line too.
            trackLineLabel(c);

            // Set the chart image to the WinChartViewer
            viewer.Chart = c;
        }

        //
        // Draw track line with data labels
        //
        private void trackLineLabel(XYChart c)
        {
            // Clear the current dynamic layer and get the DrawArea object to draw on it.
            DrawArea d = c.initDynamicLayer();

            // In this example, we have two track lines.
            const int trackLineCount = 2;

            if (trackLinePos.Count == 0)
            {
                // Initialize the track line position by distributing them on the plot area
                PlotArea p = c.getPlotArea();
                for (int i = 0; i < trackLineCount; ++i)
                    trackLinePos.Add(p.getLeftX() + (int)(p.getWidth() * (i + 0.5) / trackLineCount));
            }

            // Record the positions with the track lines
            var trackLineLog = new Dictionary<string, double>[trackLineCount];

            // Draw the track lines if enabled
            drawTrackLine(c, trackLinePos[0], trackLineLog[0] = new Dictionary<string, double>());
            drawTrackLine(c, trackLinePos[1], trackLineLog[1] = new Dictionary<string, double>());

            // Draw the differences beteween the first two track lines
            drawTrackDiff(c, trackLineLog[0], trackLineLog[1]);
        }

        void drawTrackLine(XYChart c, int lineX, Dictionary<string, double> log)
        {
            // The drawarea and plotarea objects
            DrawArea d = c.getDrawArea();
            PlotArea plotArea = c.getPlotArea();

            // Get the data x-value that is nearest to the mouse, and find its pixel coordinate.
            double xValue = c.getNearestXValue(lineX);
            int xCoor = c.getXCoor(xValue);

            // Draw empty track line if it is ahead of the data
            if ((currentIndex <= 0) || ((xCoor < lineX) && (xValue >= chartTimeLimit)))
            {
                d.vline(plotArea.getTopY(), plotArea.getBottomY(), lineX, 0x888888);
                return;
            }

            // Draw a vertical track line at the x-position
            d.vline(plotArea.getTopY(), plotArea.getBottomY(), xCoor, 0x888888);

            // Draw a label on the x-axis to show the track line position.
            string xlabel = "<*font,bgColor=000000*> " + c.xAxis().getFormattedLabel(xValue, "nn:ss.ff") +
                " <*/font*>";
            TTFText t = d.text(xlabel, "Arial Bold", 10);
            log["x"] = xValue;

            // Restrict the x-pixel position of the label to make sure it stays inside the chart image.
            int xLabelPos = Math.Max(0, Math.Min(xCoor - t.getWidth() / 2, c.getWidth() - t.getWidth()));
            t.draw(xLabelPos, plotArea.getBottomY() + 6, 0xffffff);

            // Iterate through all layers to draw the data labels
            for (int i = 0; i < c.getLayerCount(); ++i)
            {
                Layer layer = c.getLayerByZ(i);

                // The data array index of the x-value
                int xIndex = layer.getXIndexOf(xValue);

                // Iterate through all the data sets in the layer
                for (int j = 0; j < layer.getDataSetCount(); ++j)
                {
                    ChartDirector.DataSet dataSet = layer.getDataSetByZ(j);

                    // Get the color and position of the data label
                    int color = dataSet.getDataColor();
                    int yCoor = c.getYCoor(dataSet.getPosition(xIndex), dataSet.getUseYAxis());

                    // Draw a track dot with a label next to it for visible data points in the plot area
                    if ((yCoor >= plotArea.getTopY()) && (yCoor <= plotArea.getBottomY()) && (color !=
                        Chart.Transparent) && (!string.IsNullOrEmpty(dataSet.getDataName())))
                    {
                        d.circle(xCoor, yCoor, 4, 4, color, color);

                        string label = "<*font,bgColor=" + color.ToString("x") + "*> " + c.formatValue(
                            dataSet.getValue(xIndex), "{value|P4}") + " <*/font*>";
                        t = d.text(label, "Arial Bold", 10);
                        log[dataSet.getDataName()] = dataSet.getValue(xIndex);

                        // Draw the label on the right side of the dot if the mouse is on the left side the
                        // chart, and vice versa. This ensures the label will not go outside the chart image.
                        if (xCoor <= (plotArea.getLeftX() + plotArea.getRightX()) / 2)
                            t.draw(xCoor + 5, yCoor, 0xffffff, Chart.Left);
                        else
                            t.draw(xCoor - 5, yCoor, 0xffffff, Chart.Right);
                    }
                }
            }
        }
        //
        // Draw the differences between the track lines
        //
        void drawTrackDiff(XYChart c, Dictionary<string, double> log0, Dictionary<string, double> log1)
        {
            double x0, x1;
            if (!((null != log0) && log0.TryGetValue("x", out x0) && (null != log1) && log1.TryGetValue("x", out x1)))
                return;

            // Two columns in the table
            var leftCol = new System.Text.StringBuilder();
            var rightCol = new System.Text.StringBuilder();

            leftCol.Append("Change in x: ");
            rightCol.Append(c.formatValue(x1 - x0, "{value|2}"));

            // Iterate through all layers to draw the data labels
            for (int i = 0; i < c.getLayerCount(); ++i)
            {
                Layer layer = c.getLayerByZ(i);

                // Iterate through all the data sets in the layer
                for (int j = 0; j < layer.getDataSetCount(); ++j)
                {
                    var dataSetName = layer.getDataSet(j).getDataName();

                    double v0, v1;
                    if (!(log0.TryGetValue(dataSetName, out v0) && log1.TryGetValue(dataSetName, out v1)))
                        continue;
                    leftCol.Append("\nChange in ").Append(dataSetName).Append(": ");
                    rightCol.Append("\n").Append(c.formatValue(v1 - v0, "{value|2}"));
                }
            }

            string table = "<*block,bgColor=80ffffff,margin=4*><*block*>" + leftCol.ToString() +
                "<*/*><*block,halign=right*>" + rightCol.ToString() + "<*/*><*/*>";

            TTFText t = c.getDrawArea().text(table, "Arial", 10);
            t.draw(c.getPlotArea().getRightX() - t.getWidth(), c.getPlotArea().getTopY(), 0x000000);
        }

        private void hScrollBar1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            var viewer = WPFChartViewer1;

            // When the view port is changed (user drags on the chart to scroll), the scroll bar will get
            // updated. When the scroll bar changes (eg. user drags on the scroll bar), the view port will
            // get updated. This creates an infinite loop. To avoid this, the scroll bar can update the 
            // view port only if the view port is not updating the scroll bar.
            if (!viewer.IsInViewPortChangedEvent)
            {
                // Set the view port based on the scroll bar
                viewer.ViewPortLeft = (hScrollBar1.Value - hScrollBar1.Minimum)
                    / (hScrollBar1.Maximum - hScrollBar1.Minimum) * (1 - viewer.ViewPortWidth);

                // Trigger a view port changed event to update the chart
                viewer.updateViewPort(true, false);
            }
        }
        //
        // Draw track cursor when mouse is moving over plotarea
        //
        private void WPFChartViewer1_MouseMovePlotArea(object sender, MouseEventArgs e)
        {
            var viewer = sender as WPFChartViewer;

            // Mouse can drag the track lines if it is in scroll mode
            var mouseUsage = viewer.MouseUsage;
            if (((mouseUsage != WinChartMouseUsage.ScrollOnDrag) && (mouseUsage != WinChartMouseUsage.Default))
                || (trackLinePos.Count == 0))
                return;

            int mouseX = viewer.ChartMouseX;

            // Check if mouse button is down
            if (Mouse.LeftButton == MouseButtonState.Pressed)
            {
                // If mouse is near track line, then it is dragging the track line
                if (nearestTrackLine >= 0)
                {
                    XYChart c = (XYChart)viewer.Chart;
                    PlotArea p = c.getPlotArea();

                    // move the track line while ensuring the track line is in the plot area
                    trackLinePos[nearestTrackLine] =
                        Math.Min(p.getRightX(), Math.Max(p.getLeftX(), mouseX - nearestDistance));

                    // repaint the track lines
                    trackLineLabel(c);
                    viewer.updateDisplay();
                }
            }
            else
            {
                // Check which track line is nearest to the mouse
                nearestTrackLine = -1;
                nearestDistance = 9;
                for (int i = 0; i < trackLinePos.Count; ++i)
                {
                    if (Math.Abs(mouseX - trackLinePos[i]) < Math.Abs(nearestDistance))
                    {
                        nearestTrackLine = i;
                        nearestDistance = mouseX - trackLinePos[i];
                    }
                }

                // If mouse is near the track line, it is used to drag the line, so disable drag to scroll.
                viewer.MouseUsage = ((nearestTrackLine >= 0) ? WinChartMouseUsage.Default :
                    WinChartMouseUsage.ScrollOnDrag);
            }
        }

        #endregion
        private void OnPropertyChanged(String info)
        {
            PropertyChangedEventHandler handler = PropertyChanged;
            if (handler != null)
            {
                handler(this, new PropertyChangedEventArgs(info));
            }
        }

        #region Function of Slider
        public int HueLow
        {
            get
            { return hueLow; }
            set
            { hueLow = value; }
        }

        public int HueHigh
        {
            get
            { return hueHigh; }
            set
            { hueHigh = value; }
        }

        public int SatLow
        {
            get
            { return satLow; }
            set
            { satLow = value; }
        }
        public int SatHigh
        {
            get
            { return satHigh; }
            set
            { satHigh = value; }
        }
        public int ValLow
        {
            get
            { return valLow; }
            set
            { valLow = value; }
        }
        public int ValHigh
        {
            get
            { return valHigh; }
            set
            { valHigh = value; }
        }

        public int PX
        {
            get
            { return valPX; }
            set
            { valPX = value; }
        }

        public int IX
        {
            get
            { return valIX; }
            set
            { valIX = value; }
        }

        public int DX
        {
            get
            { return valDX; }
            set
            { valDX = value; }
        }

        public int PY
        {
            get
            { return valPY; }
            set
            { valPY = value; }
        }

        public int IY
        {
            get
            { return valIY; }
            set
            { valIY = value; }
        }

        public int DY
        {
            get
            { return valDY; }
            set
            { valDY = value; }
        }
        public int PosX
        {
            get { return posX; }
            set
            {
                if (PosX != value)
                {
                    posX = value;
                    OnPropertyChanged("PosX");
                }
            }
        }
        //public int PosX { get => posX; set => posX = value; }
        public int PosY
        {
            get { return posY; }
            set
            {
                if (PosY != value)
                {
                    posY = value;
                    OnPropertyChanged("PosY");
                }
            }
        }
        public int Alpha
        {
            get { return alpha; }
            set
            {
                if (Alpha != value)
                {
                    alpha = value;
                    OnPropertyChanged("Alpha");
                }
            }
        }
        public int Beta
        {
            get { return beta; }
            set
            {
                if (Beta != value)
                {
                    beta = value;
                    OnPropertyChanged("Beta");
                }
            }
        }
        public int SetPointX
        {
            get { return setpointx; }
            set
            {
                if (setpointx != value)
                {
                    setpointx = value;
                    OnPropertyChanged("SetPointX");
                }
            }
        }
        public int SetPointY
        {
            get { return setpointy; }
            set
            {
                if (setpointy != value)
                {
                    setpointy = value;
                    OnPropertyChanged("SetPointY");
                }
            }
        }

        public void ProduceThresholdImage()
        {
            HueLow = (int)(tBar_HLo.Value);
            HueHigh = (int)tBar_HHi.Value;
            SatLow = (int)tBar_SLo.Value;
            SatHigh = (int)tBar_SHi.Value;
            ValLow = (int)tBar_VLo.Value;
            ValHigh = (int)tBar_VHi.Value;

            PX = (int)tBar_PX.Value;
            IX = (int)tBar_IX.Value;
            DX = (int)tBar_DX.Value;
            PY = (int)tBar_PY.Value;
            IY = (int)tBar_IY.Value;
            DY = (int)tBar_DY.Value;

            SetPointX = (int)tBar_SetPointX.Value;
            SetPointY = (int)tBar_SetPointY.Value;

            tbl_HueLow.Text = HueLow.ToString();
            tbl_HueHigh.Text = HueHigh.ToString();
            tbl_SatLow.Text = SatLow.ToString();
            tbl_SatHigh.Text = SatHigh.ToString();
            tbl_ValLow.Text = ValLow.ToString();
            tbl_ValHigh.Text = ValHigh.ToString();
            tbl_PX.Text = PX.ToString();
            tbl_IX.Text = IX.ToString();
            tbl_DX.Text = DX.ToString();
            tbl_PY.Text = PY.ToString();
            tbl_IY.Text = IY.ToString();
            tbl_DY.Text = DY.ToString();
            tbl_SetPointX.Text = SetPointX.ToString();
            tbl_SetPointY.Text = SetPointY.ToString();
        }

        private void tBar_VHi_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }

        private void tBar_VLo_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }

        private void tBar_SHi_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }

        private void tBar_SLo_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }

        private void tBar_HHi_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }

        private void tBar_HLo_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }
        #endregion

        private void tBar_SetPointX_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }
        int Check = 0;
        private void RadioSetPoint_Checked(object sender, RoutedEventArgs e)
        {
            Check = 0;
        }

        private void RadioTriangle_Checked(object sender, RoutedEventArgs e)
        {
            nFlag = 0;
            tStart = Stopwatch.StartNew();
            Check = 1;
        }

        private void RadioRectangle_Checked(object sender, RoutedEventArgs e)
        {
            nFlag = 0;
            tStart = Stopwatch.StartNew();
            Check = 2;
        }

        private void RadioCircle_Checked(object sender, RoutedEventArgs e)
        {
            nFlag = 0;
            tStart = Stopwatch.StartNew();
            Check = 3;
        }
        int HomeX = 104;
        int HomeY = 57;
        private void tBar_SetPointY_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ProduceThresholdImage();
        }

        private delegate void UpdateImageDelegate(List<Mat> imgList);

        private void ProcessFrame(object sender, EventArgs e)
        {
            //sw = Stopwatch.StartNew();
            //Image<Bgr, byte> imgCon = new Image<Bgr, byte>(_capture.Width, _capture.Height);

            Image<Gray, byte> imgThresholded = new Image<Gray, byte>(_capture.Width, _capture.Height);
            Image<Gray, byte> imgGray2 = new Image<Gray, byte>(_capture.Width, _capture.Height);
            //Image<Bgr, byte> imgCon = imgGray.Convert<Bgr, byte>();
            //UMat imgHSV = new UMat();
            //UMat imgGray = new UMat();


            Mat hier = new Mat();
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();

            try
            {
                _capture.Retrieve(frame);
                System.Drawing.Point pCenter = new System.Drawing.Point(frame.Width / 2, frame.Height / 2);
                System.Drawing.Rectangle RectROI = new System.Drawing.Rectangle(pCenter.X - 215, pCenter.Y - 225, 435, 435);


                //CvInvoke.CvtColor(frame, imgGray, ColorConversion.Bgr2Gray);
                Image<Hsv, byte> imgHSV = new Image<Hsv, byte>(_capture.Width, _capture.Height);

                CvInvoke.CvtColor(frame, imgHSV, ColorConversion.Rgb2Hsv);

                CvInvoke.Rectangle(frame, RectROI, new MCvScalar(0, 255, 0), 2);
                
                imgHSV.ROI = RectROI;

                int nOffSetX = RectROI.X;
                int nOffSetY = RectROI.Y;

                Hsv hsv_min = new Hsv(HueLow, SatLow, ValLow);        //just care about color channel
                Hsv hsv_max = new Hsv(HueHigh, SatHigh, ValHigh);

                CvInvoke.Line(frame, new System.Drawing.Point(pCenter.X - 30 , pCenter.Y), new System.Drawing.Point(pCenter.X + 30, pCenter.Y), new MCvScalar(255, 0, 0), 2);
                CvInvoke.Line(frame, new System.Drawing.Point(pCenter.X, pCenter.Y - 30), new System.Drawing.Point(pCenter.X, pCenter.Y + 30), new MCvScalar(255, 0, 0), 2);
                imgThresholded = imgHSV.InRange(hsv_min, hsv_max);

                Mat kernel1 = CvInvoke.GetStructuringElement(Emgu.CV.CvEnum.ElementShape.Ellipse, new System.Drawing.Size(10, 10), new System.Drawing.Point(1, 1));
                Image<Gray, byte> tmp2 = imgThresholded.MorphologyEx(MorphOp.Erode, kernel1, new System.Drawing.Point(-1, -1), 1, BorderType.Default, new MCvScalar());

                var M = CvInvoke.Moments(tmp2);

                dArea = M.M00;
                if (dArea > 5000)
                {
                    int posBallX = (int)(M.M10 / dArea);
                    int posBallY = (int)(M.M01 / dArea);
                    PosX = (posBallX - pCenter.X + nOffSetX) * 51 / 73;
                    PosY = -(posBallY - pCenter.Y + nOffSetY) * 51 / 73;
                    CvInvoke.Line(frame, new System.Drawing.Point(posBallX - 30 + nOffSetX, posBallY + nOffSetY), new System.Drawing.Point(posBallX + 30 + nOffSetX, posBallY + nOffSetY), new MCvScalar(0, 0, 255), 2);
                    CvInvoke.Line(frame, new System.Drawing.Point(posBallX + nOffSetX, posBallY - 30 + nOffSetY), new System.Drawing.Point(posBallX + nOffSetX, posBallY + 30 + nOffSetY), new MCvScalar(0, 0, 255), 2);
                }

                img_Raw_box.Dispatcher.Invoke(() =>
                {
                    img_Raw_box.Source = BitmapSourceConvert.ToBitmapSource(frame);
                }
                );


                img_Bin_box.Dispatcher.Invoke(() =>
                {
                    img_Bin_box.Source = BitmapSourceConvert.ToBitmapSource(tmp2);
                }
                );

            }
            catch (Exception exception)
            {

                System.Windows.MessageBox.Show(exception.ToString());
            }
        }



        private void Connect_Btn_Click(object sender, RoutedEventArgs e)
        {
            if ((string)Status.Badge == "No Connect")
            {
                if(Comm_Port_Names.Text == "")
                {

                }
                else
                {
                    //Sets up serial port
                    serial.PortName = Comm_Port_Names.Text;
                    serial.BaudRate = 9600;
                    serial.Handshake = System.IO.Ports.Handshake.None;
                    serial.Parity = Parity.None;
                    serial.DataBits = 8;
                    serial.StopBits = StopBits.One;
                    serial.ReadTimeout = 200;
                    serial.WriteTimeout = 50;
                    serial.Open();

                    //Sets button State and Creates function call on data recieved
                    Connect_Btn.Content = "Disconnect";
                    Status.Badge = "Connected";
                    Status.BadgeColorZoneMode = MaterialDesignThemes.Wpf.ColorZoneMode.Accent;
                }
            }
            else
            {
                try // just in case serial port is not open could also be acheved using if(serial.IsOpen)
                {
                    serial.Close();
                    Connect_Btn.Content = "Connect";
                    Status.Badge = "No Connect";
                    Status.BadgeColorZoneMode = MaterialDesignThemes.Wpf.ColorZoneMode.Dark;
                }
                catch
                {
                }
            }
        }

        public void SerialCmdSend(string data)
        {
            if (serial.IsOpen)
            {
                try
                {
                    // Send the binary data out the port
                    byte[] hexstring = Encoding.ASCII.GetBytes(data);
                    //There is a intermitant problem that I came across
                    //If I write more than one byte in succesion without a 
                    //delay the PIC i'm communicating with will Crash
                    //I expect this id due to PC timing issues ad they are
                    //not directley connected to the COM port the solution
                    //Is a ver small 1 millisecound delay between chracters
                    foreach (byte hexval in hexstring)
                    {
                        byte[] _hexval = new byte[] { hexval }; // need to convert byte to byte[] to write
                        serial.Write(_hexval, 0, 1);
                        Thread.Sleep(1);
                    }
                }
                catch (Exception ex)
                {
                    para.Inlines.Add("Failed to SEND" + data + "\n" + ex + "\n");
                    mcFlowDoc.Blocks.Add(para);
                }
            }
            else
            {
            }
        }
    }
}

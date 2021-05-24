/*
 * The application tracks right hand. Recognize gesture of left hand. 
 * Displays the hands movement and joint position. 
 * Displays depth image, color image and skeleton tracking. Sends data to robotic arm.
 */

namespace TrackingHand
{
    using System;
    using System.Linq;
    using System.Windows;
    using System.Windows.Controls;
    using Microsoft.Kinect;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.IO;

    // Libraries for TCP/IP, Threading and String
    using System.Text;
    using System.Net.Sockets;
    using System.Threading;
       
    // Interaction logic for MainWindow.xaml
    public partial class MainWindow : Window
    {
        // Define the kinect sensor
        private KinectSensor sensor;

        // TCP/IP client initialization
        TcpClient tcpclnt = new TcpClient();

        // String initialization
        public String str = "0";

        private MainWindowViewModel viewModel;

        public bool IsEmitterOn { get; set; }

        // Gets or sets the pixel data
        private byte[] pixelData { get; set; }

        // Total number of skeleton can be tracked
        Skeleton[] totalSkeleton = new Skeleton[6];

        // Initializes a new instance of the MainWindow class
        public MainWindow()
        {
            InitializeComponent();

            // Threading tcp/ip client read/write data
            Thread thread = new Thread(new ThreadStart(Connect));
            thread.Start();

            this.viewModel = new MainWindowViewModel();
            this.DataContext = this.viewModel;
            Loaded += new RoutedEventHandler(MainWindow_Loaded);
            Unloaded += new RoutedEventHandler(MainWindow_Unloaded);
        }

        // Handles the Unloaded event of the MainWindow control
        void MainWindow_Unloaded(object sender, RoutedEventArgs e)
        {
            this.sensor.Stop();
        }
        
        // Function for TCP/IP client connection
        public void Connect()
        {
            while (true)
            {
                TcpClient tcpclnt = new TcpClient();

                try
                {
                    tcpclnt.Connect("IPadressHERE", 6660);
                    Stream stm = tcpclnt.GetStream();
                    ASCIIEncoding asen = new ASCIIEncoding();
                    byte[] ba = asen.GetBytes(str);
                    byte[] br = new byte[1024];

                    stm.Write(ba, 0, ba.Length);

                    System.Threading.Thread.Sleep(50);

                    //stm.Read(br, 0, br.Length);
                    //string_data = Encoding.ASCII.GetString(data, 0, recv);
                    //Console.WriteLine(br);
                }
                catch
                {
                    break;
                }
                finally
                {
                    tcpclnt.Close();
                }
            }
        }

        protected void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // Checks if there is kinect connected
            if (KinectSensor.KinectSensors.Count > 0)
            {
                // Gets the first kinect sensor
                this.sensor = KinectSensor.KinectSensors[0];

                // Starts the sensor
                this.sensor.Start();

                // Checks if the color and depth stream is enabled. If not, then it enables and attaches the event handler
                this.sensor.ColorStream.Disable();
                if (!this.sensor.ColorStream.IsEnabled)
                {
                    this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                    this.sensor.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(sensor_ColorFrameReady2);
                    this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                    this.sensor.DepthFrameReady += Sensor_DepthFrameReady;
                }
            }
            else
            {
                MessageBox.Show("No Device Connected");
                Application.Current.Shutdown();
            }

            // Checks if the skeleton stream is enabled. If not, then it enables and attaches the event handler
            if (!this.sensor.SkeletonStream.IsEnabled)
            {
                this.sensor.SkeletonStream.Enable();
                this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                this.sensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(sensor_SkeletonFrameReady);
            }
            // Create the drawing group used for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that can is used in image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make app robust against plug/unplug, 
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }
            
            // Smoothing parametres for skeleton tracking
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames with smoothed parameters
                this.sensor.SkeletonStream.Enable(parameters);

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor again to ensure it is running
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                MessageBox.Show("No Device Connected");
            }
        }

        // Executes shutdown tasks 
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        // Event handler for Kinect sensor's SkeletonFrameReady event
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draws a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }
                // Prevents drawing outside of the render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        // Called when the skeleton's frame is ready
        void sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                // Checks for frame drop
                if (skeletonFrame == null)
                {
                    return;
                }
                // Copies the frame data in to the collection
                skeletonFrame.CopySkeletonDataTo(totalSkeleton);

                // Gets the first Tracked skeleton
                Skeleton firstSkeleton = (from trackskeleton in totalSkeleton
                                          where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                                          select trackskeleton).FirstOrDefault();

                // If the first skeleton returns null
                if (firstSkeleton == null)
                {
                    return;
                }

                // It is checking for Right Hand, check if Right Hand Tracking State. Calls the mapping method only if the joint is tracked.
                if (firstSkeleton.Joints[JointType.HandRight].TrackingState == JointTrackingState.Tracked)
                {
                    this.SkeletonTracking(firstSkeleton);
                }
            }
        }

        // Calculating data for a robotic arm
        private void SkeletonTracking(Skeleton skeleton)
        {
            Point mappedPoint = this.ScalePosition(skeleton.Joints[JointType.HandRight].Position);
            Point leftmappedPoint = this.ScalePosition(skeleton.Joints[JointType.HandLeft].Position);

            SkeletonPoint skeletonPointR = skeleton.Joints[JointType.HandRight].Position;
            float xr = skeletonPointR.X;
            float yr = skeletonPointR.Y;
            float zr = skeletonPointR.Z;
            SkeletonPoint skeletonPointS = skeleton.Joints[JointType.ShoulderCenter].Position;
            float xs = skeletonPointS.X;
            float ys = skeletonPointS.Y;
            float zs = skeletonPointS.Z;
            var X = (xr - xs - 0.25) * 1000;
            var Y = (yr - ys + 0.1) * 1000;
            var Z = (zs - zr - 0.15) * 1000;
            double Xz = Math.Round(X,0);
            double Yz = Math.Round(Y,0);
            double Zz = Math.Round(Z,0);

            SkeletonPoint skeletonPointL = skeleton.Joints[JointType.HandLeft].Position;
            float xl = skeletonPointL.X * 1000;
            float yl = skeletonPointL.Y * 1000;
            float zl = skeletonPointL.Z * 1000;
            SkeletonPoint skeletonPointE = skeleton.Joints[JointType.ElbowLeft].Position;
            float ye = skeletonPointE.Y * 1000;

            double xlz = Math.Round(xl, 0);
            double ylz = Math.Round(yl, 0);
            double zlz = Math.Round(zl, 0);

            // Gesture recognition
            var Yl = (yl - ye) * 10;
            int move;
            string moveWord;
            if (Yl > 0)
            {
                move = 1;
                moveWord = "start";

            }
            else
            {
                move = 0;
                moveWord = "stop";
            }

            // Result string ready to be sent to the robotic arm
            str = "[" + Xz + "," + Yz + "," + Zz + "," + move.ToString() + "]";

            // Righthand
            myhandPosition.Content = string.Format("X:{0},Y:{1},Z:{2}", Xz, Yz, Zz);
            robotData.Content = string.Format("[X:{0}, Y:{1}, Z:{2}]  Move: {3}", Xz, Yz, Zz,moveWord);
            Canvas.SetLeft(righthand, mappedPoint.X);
            Canvas.SetTop(righthand, mappedPoint.Y);
           
            // Lefthand
            mylefthandPosition.Content = string.Format("X:{0},Y:{1}, Z:{2}", xlz, ylz, zlz);
            Canvas.SetLeft(lefthand, leftmappedPoint.X);
            Canvas.SetTop(lefthand, leftmappedPoint.Y);
        }

        // Scales the position.
        private Point ScalePosition(SkeletonPoint skeletonPoint)
        {
            // Returns the depth points from the skeleton point
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeletonPoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        // Handles the ValueChanged event of the Slider control.
        private void Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            this.SetSensorAngle(int.Parse(e.NewValue.ToString()));
        }

        // Sets the sensor angle.
        private void SetSensorAngle(int angleValue)
        {
            if (angleValue > this.sensor.MinElevationAngle || angleValue < this.sensor.MaxElevationAngle)
            {
                this.viewModel.SensorAngle = angleValue;
                this.sensor.ElevationAngle = this.viewModel.SensorAngle;
            }
        }

        // Width of output drawing
        private const float RenderWidth = 640.0f;

        // Height of our output drawing
        private const float RenderHeight = 480.0f;

        // Thickness of drawn joint lines
        private const double JointThickness = 3;

        // Thickness of body center ellipse
        private const double BodyCenterThickness = 10;

        // Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;

        // Brush used to draw skeleton center point
        private readonly Brush centerPointBrush = Brushes.Blue;

        // Brush used for drawing joints that are currently tracked
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        // Brush used for drawing joints that are currently inferred
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        // Pen used for drawing bones that are currently tracked
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        // Pen used for drawing bones that are currently inferred
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        // Drawing group for skeleton rendering output
        private DrawingGroup drawingGroup;

        // Drawing image that we will display
        private DrawingImage imageSource;

        // Draws indicators to show which edges are clipping skeleton data
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        // Draws a skeleton's bones and joints
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        // Maps a SkeletonPoint to lie within our render space and converts to Point
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Converts point to color space
           ColorImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelpoint, ColorImageFormat.RgbResolution640x480Fps30);        
           return new Point(depthPoint.X, depthPoint.Y);
        }

        // Draws a bone line between two joints
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // Assumed that all drawn bones are inferred unless both joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        // Creates bitmap of color image
        void sensor_ColorFrameReady2(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame imageFrame = e.OpenColorImageFrame())
            {
                // Check if the incoming frame is not null
                if (imageFrame != null)
                {
                    // Get the pixel data in byte array
                    this.pixelData = new byte[imageFrame.PixelDataLength];


                    // Copy the pixel data
                    imageFrame.CopyPixelDataTo(this.pixelData);

                    // Assigns the bitmap image source into image control
                    BitmapSource bitmapS = BitmapSource.Create(
                     imageFrame.Width,
                     imageFrame.Height,
                     96,
                     96,
                     PixelFormats.Bgr32,
                     null,
                     pixelData,
                     imageFrame.Width * 4);

                    this.colorimageControl.Source = bitmapS;

                }
            }
        }

        // Defines max and min distance for depth image
        const float MaxDepthDistance = 4000;
        const float MinDepthDistance = 850;
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;
        
        // Creates bitmap of depth image
        private void Sensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame == null)
                {
                    return;
                }
                byte[] pixels = GenerateColoredBytes(depthFrame);
                int stride = depthFrame.Width * 4;
                depthimage.Source = BitmapSource.Create(depthFrame.Width, depthFrame.Height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
            }
        }

        // Coloring of depth image
        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];

            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                byte intensity = CalculateIntensityFromDepth(depth);

                pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;

                if (player > 0)
                {
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = intensity;
                    pixels[colorIndex + RedIndex] = 0;

                }
            }
            return pixels;
        }

        public static byte CalculateIntensityFromDepth(int distance)
        {
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0) / (MaxDepthDistanceOffset)));
        }



    }
}

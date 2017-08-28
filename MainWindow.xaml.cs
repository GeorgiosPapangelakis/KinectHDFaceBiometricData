// --------------------------------------------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// --------------------------------------------------------------------------------------------------------------------
namespace Microsoft.Samples.Kinect.HDFaceBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Windows.Media.Media3D;
    using Microsoft.Kinect.Face;

    /// <summary>
    /// Main Window
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged, IDisposable
    {
        string MyFile = "C:\\Users\\Georgios Papangelaki\\Desktop\\HDFaceBasics-WPF\\KinectHDFaceFrameReader.txt";
        System.IO.StreamWriter Writer;

        int totalFrames = 0;
        string TOPRINT = "";
        int FrameBlink = 0;
        int BetweenBlink = 0;
        int EndBlink = 0;
        public List<TimeSpan> Clock;
        public List<List<CameraSpacePoint>> PointsData;
        public List<CameraSpacePoint> TempList;
        public List<int> LEyeIndex;
        public List<int> REyeIndex;
        public List<int> FrameBetFreq;

        public struct BiometricData
        {
            public String Name;
            public int TestSubject;
            public double FreqOfBlinks;
            public List<List<List<Double>>> SpeedOfBlinks;
            public List<List<List<Double>>> AccelOfBlinks;
        }
        BiometricData NewSubject;

        /// <summary>
        /// Currently used KinectSensor
        /// </summary>
        private KinectSensor sensor = null;

        /// <summary>
        /// Body frame source to get a BodyFrameReader
        /// </summary>
        private BodyFrameSource bodySource = null;

        /// <summary>
        /// Body frame reader to get body frames
        /// </summary>
        private BodyFrameReader bodyReader = null;

        /// <summary>
        /// HighDefinitionFaceFrameSource to get a reader and a builder from.
        /// Also to set the currently tracked user id to get High Definition Face Frames of
        /// </summary>
        private HighDefinitionFaceFrameSource highDefinitionFaceFrameSource = null;

        /// <summary>
        /// HighDefinitionFaceFrameReader to read HighDefinitionFaceFrame to get FaceAlignment
        /// </summary>
        private HighDefinitionFaceFrameReader highDefinitionFaceFrameReader = null;

        private FaceFrameSource faceFrameSource = null;

        /// <summary>
        /// Face frame readers
        /// </summary>
        private FaceFrameReader faceFrameReader = null;

        /// <summary>
        /// Storage for face frame results
        /// </summary>
        private FaceFrameResult faceFrameResult = null;
        private int bodyCount;

        private CoordinateMapper coordinateMapper = null;



        /// <summary>
        /// FaceAlignment is the result of tracking a face, it has face animations location and orientation
        /// </summary>
        private FaceAlignment currentFaceAlignment = null;

        /// <summary>
        /// FaceModel is a result of capturing a face
        /// </summary>
        private FaceModel currentFaceModel = null;

        /// <summary>
        /// FaceModelBuilder is used to produce a FaceModel
        /// </summary>
        private FaceModelBuilder faceModelBuilder = null;

        /// <summary>
        /// The currently tracked body
        /// </summary>
        private Body currentTrackedBody = null;

        /// <summary>
        /// The currently tracked body
        /// </summary>
        private ulong currentTrackingId = 0;

        /// <summary>
        /// Gets or sets the current tracked user id
        /// </summary>
        private string currentBuilderStatus = string.Empty;

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        private string statusText = "Ready To Start Capture";

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            this.InitializeComponent();
            this.DataContext = this;
            this.Clock = new List<TimeSpan>();
            PointsData = new List<List<CameraSpacePoint>>();
            Writer = new System.IO.StreamWriter(MyFile, true);
            LEyeIndex = new List<int> { 113, 121, 123, 137, 153, 185, 187, 188, 211, 212, 229, 230, 234, 236, 237, 238, 240, 241, 242, 243, 244, 265, 287, 316, 325, 326, 327, 328, 330, 331, 332, 333, 341, 343, 348, 349, 350, 467, 1075, 1103, 1104, 1105, 1106, 1107, 1108, 1109, 1111, 1112, 1113, 1114 };
            REyeIndex = new List<int> { 728, 729, 730, 731, 732, 733, 749, 750, 751, 752, 775, 776, 777, 778, 821, 826, 841, 842, 845, 846, 862, 863, 865, 869, 870, 871, 873, 874, 875, 876, 877, 882, 883, 925, 985, 986, 987, 992, 1089, 1090, 1091, 1092, 1093, 1094, 1095, 1096, 1097, 1099, 1100, 1101 };
            FrameBetFreq = new List<int>();
            NewSubject.SpeedOfBlinks = new List<List<List<double>>>();
            NewSubject.AccelOfBlinks = new List<List<List<double>>>();
            NewSubject.Name = "TO SET";
            NewSubject.TestSubject = 1;
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Gets or sets the current tracked user id
        /// </summary>
        private ulong CurrentTrackingId
        {
            get
            {
                return this.currentTrackingId;
            }

            set
            {
                this.currentTrackingId = value;

                this.StatusText = this.MakeStatusText();
            }
        }

        /// <summary>
        /// Gets or sets the current Face Builder instructions to user
        /// </summary>
        private string CurrentBuilderStatus
        {
            get
            {
                return this.currentBuilderStatus;
            }

            set
            {
                this.currentBuilderStatus = value;

                this.StatusText = this.MakeStatusText();
            }
        }

        /// <summary>
        /// Called when disposed of
        /// </summary>
        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Dispose based on whether or not managed or native resources should be freed
        /// </summary>
        /// <param name="disposing">Set to true to free both native and managed resources, false otherwise</param>
        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (this.currentFaceModel != null)
                {
                    this.currentFaceModel.Dispose();
                    this.currentFaceModel = null;
                }
            }
        }

        /// <summary>
        /// Returns the length of a vector from origin
        /// </summary>
        /// <param name="point">Point in space to find it's distance from origin</param>
        /// <returns>Distance from origin</returns>
        private static double VectorLength(CameraSpacePoint point)
        {
            var result = Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2);

            result = Math.Sqrt(result);

            return result;
        }

        private double DistBetPoints(CameraSpacePoint P1, CameraSpacePoint P2)
        {
            return Math.Sqrt(((P2.X - P1.X) * (P2.X - P1.X)) + ((P2.Y - P1.Y) * (P2.Y - P1.Y)) + ((P2.Z - P1.Z) * (P2.Z - P1.Z)));
        }

        private double SpeedBetPoints(CameraSpacePoint P1, CameraSpacePoint P2)
        {
            return DistBetPoints(P1, P2) / 0.0333333333;
        }
        private double AccelBetPoints(CameraSpacePoint P1, CameraSpacePoint P2, CameraSpacePoint P3)
        {
            return (SpeedBetPoints(P2, P3) - SpeedBetPoints(P1, P2)) / 0.0333333333;
        }

        private List<List<Double>> CalculateAccel(List<List<CameraSpacePoint>> PData)
        {
            List<List<Double>> APoints = new List<List<double>>();
            for (int i = 0; i < (PData.Count) - 2; i++)
            {
                List<Double> TList = new List<Double>();
                List<CameraSpacePoint> P1 = PData[i];
                List<CameraSpacePoint> P2 = PData[i + 1];
                List<CameraSpacePoint> P3 = PData[i + 2];
                for (int j = 0; j < (PData[0].Count); j++)
                {
                    TList.Add(AccelBetPoints(P1[j], P2[j], P3[j]));
                }
                APoints.Add(TList);
            }
            return APoints;
        }
        private List<List<Double>> CalculateSpeed(List<List<CameraSpacePoint>> PData)
        {
            List<List<Double>> SPoints = new List<List<double>>();
            for (int i = 0; i < (PData.Count) - 1; i++)
            {
                List<Double> TList = new List<Double>();
                List<CameraSpacePoint> P1 = PData[i];
                List<CameraSpacePoint> P2 = PData[i + 1];
                for (int j = 0; j < PData[0].Count; j++)
                {
                    TList.Add(SpeedBetPoints(P1[j], P2[j]));
                }
                SPoints.Add(TList);
            }
            return SPoints;
        }

        private double CalcFreq(List<int> Hz)
        {
            double TF = 0;
            for (int i = 0; i < Hz.Count; i++)
            {
                TF += Hz[i];
            }
            TF *= 0.03333333333;
            return (Hz.Count) / TF;
        }

        /// <summary>
        /// Finds the closest body from the sensor if any
        /// </summary>
        /// <param name="bodyFrame">A body frame</param>
        /// <returns>Closest body, null of none</returns>
        private static Body FindClosestBody(BodyFrame bodyFrame)
        {
            Body result = null;
            double closestBodyDistance = double.MaxValue;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    var currentLocation = body.Joints[JointType.SpineBase].Position;

                    var currentDistance = VectorLength(currentLocation);

                    if (result == null || currentDistance < closestBodyDistance)
                    {
                        result = body;
                        closestBodyDistance = currentDistance;
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Find if there is a body tracked with the given trackingId
        /// </summary>
        /// <param name="bodyFrame">A body frame</param>
        /// <param name="trackingId">The tracking Id</param>
        /// <returns>The body object, null of none</returns>
        private static Body FindBodyWithTrackingId(BodyFrame bodyFrame, ulong trackingId)
        {
            Body result = null;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    if (body.TrackingId == trackingId)
                    {
                        result = body;
                        break;
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Gets the current collection status
        /// </summary>
        /// <param name="status">Status value</param>
        /// <returns>Status value as text</returns>
        private static string GetCollectionStatusText(FaceModelBuilderCollectionStatus status)
        {
            string res = string.Empty;

            if ((status & FaceModelBuilderCollectionStatus.FrontViewFramesNeeded) != 0)
            {
                res = "FrontViewFramesNeeded";
                return res;
            }

            if ((status & FaceModelBuilderCollectionStatus.LeftViewsNeeded) != 0)
            {
                res = "LeftViewsNeeded";
                return res;
            }

            if ((status & FaceModelBuilderCollectionStatus.RightViewsNeeded) != 0)
            {
                res = "RightViewsNeeded";
                return res;
            }

            if ((status & FaceModelBuilderCollectionStatus.TiltedUpViewsNeeded) != 0)
            {
                res = "TiltedUpViewsNeeded";
                return res;
            }

            if ((status & FaceModelBuilderCollectionStatus.Complete) != 0)
            {
                res = "Complete";
                return res;
            }

            if ((status & FaceModelBuilderCollectionStatus.MoreFramesNeeded) != 0)
            {
                res = "TiltedUpViewsNeeded";
                return res;
            }

            return res;
        }

        /// <summary>
        /// Helper function to format a status message
        /// </summary>
        /// <returns>Status text</returns>
        private string MakeStatusText()
        {
            string status = string.Format(System.Globalization.CultureInfo.CurrentCulture, "Builder Status: {0}, Current Tracking ID: {1}", this.CurrentBuilderStatus, this.CurrentTrackingId);

            return status;
        }

        /// <summary>
        /// Fires when Window is Loaded
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            this.InitializeHDFace();
        }

        /// <summary>
        /// Initialize Kinect object
        /// </summary>
        private void InitializeHDFace()
        {
            this.CurrentBuilderStatus = "Ready To Start Capture";

            this.sensor = KinectSensor.GetDefault();
            this.bodySource = this.sensor.BodyFrameSource;

            this.bodyReader = this.bodySource.OpenReader();
            this.bodyReader.FrameArrived += this.BodyReader_FrameArrived;
            this.bodyCount = this.sensor.BodyFrameSource.BodyCount;

            this.highDefinitionFaceFrameSource = new HighDefinitionFaceFrameSource(this.sensor);
            this.highDefinitionFaceFrameSource.TrackingIdLost += this.HdFaceSource_TrackingIdLost;

            this.highDefinitionFaceFrameReader = this.highDefinitionFaceFrameSource.OpenReader();
            this.highDefinitionFaceFrameReader.FrameArrived += this.HdFaceReader_FrameArrived;

            this.currentFaceModel = new FaceModel();
            this.currentFaceAlignment = new FaceAlignment();

            this.coordinateMapper = this.sensor.CoordinateMapper;

            FaceFrameFeatures faceFrameFeatures =
FaceFrameFeatures.BoundingBoxInColorSpace
| FaceFrameFeatures.PointsInColorSpace
| FaceFrameFeatures.RotationOrientation
| FaceFrameFeatures.FaceEngagement
| FaceFrameFeatures.Glasses
| FaceFrameFeatures.Happy
| FaceFrameFeatures.LeftEyeClosed
| FaceFrameFeatures.RightEyeClosed
| FaceFrameFeatures.LookingAway
| FaceFrameFeatures.MouthMoved
| FaceFrameFeatures.MouthOpen;


            // create the face frame source with the required face frame features and an initial tracking Id of 0
            this.faceFrameSource = new FaceFrameSource(this.sensor, 0, faceFrameFeatures);

            // open the corresponding reader
            this.faceFrameReader = this.faceFrameSource.OpenReader();


            this.faceFrameResult = null;


            // wire handler for face frame arrival
            if (this.faceFrameReader != null)
            {
                // wire handler for face frame arrival
                this.faceFrameReader.FrameArrived += this.Reader_FaceFrameArrived;
            }

            this.InitializeMesh();


            this.UpdateMesh();

            this.sensor.Open();
        }
        /// <summary>
        /// 
        ///        //////////////////////////////START OF FACE READER CODE///////////////////
        /// 
        /// 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Reader_FaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
            {
                if (faceFrame != null)
                {
                    this.faceFrameResult = faceFrame.FaceFrameResult;
                    if (this.faceFrameResult != null)
                    {
                        string RClosed = DetectionResult.Maybe.ToString();
                        string LClosed = DetectionResult.Maybe.ToString();
                        var faceProperties = this.faceFrameResult.FaceProperties;
                        foreach (var PropF in faceProperties)
                        {
                            if (PropF.Key.ToString() == "RightEyeClosed")
                            {
                                TOPRINT += PropF.Key.ToString() + " : ";
                                if (PropF.Value == DetectionResult.Maybe)
                                {
                                    TOPRINT += DetectionResult.No + Environment.NewLine;
                                }
                                else
                                {
                                    TOPRINT += PropF.Value.ToString() + Environment.NewLine;
                                }
                                RClosed = PropF.Value.ToString();
                            }
                            if (PropF.Key.ToString() == "LeftEyeClosed")
                            {
                                TOPRINT += PropF.Key.ToString() + " : ";
                                if (PropF.Value == DetectionResult.Maybe)
                                {
                                    TOPRINT += DetectionResult.No + Environment.NewLine;
                                }
                                else
                                {
                                    TOPRINT += PropF.Value.ToString() + Environment.NewLine;
                                }
                                LClosed = PropF.Value.ToString();
                            }
                        }
                        if (LClosed == "Yes" && RClosed == "Yes")
                        {
                            EndBlink = 1;
                            if (BetweenBlink > 0)
                            {
                                FrameBetFreq.Add(BetweenBlink);
                                if (FrameBetFreq.Count > 20)
                                {
                                    FrameBetFreq.RemoveAt(0);
                                }
                            }
                            BetweenBlink = 0;
                            FrameBlink += 1;
                            if (FrameBlink >= 15)
                            {
                                TOPRINT += "Not a blink, Eyes kept closed " + Environment.NewLine;
                                EndBlink = 0;
                            }
                        }
                        else
                        {
                            if (EndBlink == 1)
                            {
                                NewSubject.AccelOfBlinks.Add(CalculateAccel(PointsData));
                                TOPRINT += "Accel of Left MidTop = " + Environment.NewLine;
                                for(int i = 0; i < NewSubject.AccelOfBlinks[0].Count; i++)
                                {
                                    TOPRINT += NewSubject.AccelOfBlinks[0][i][17].ToString() + Environment.NewLine;
                                }
                                TOPRINT += "Accel of Left MidBottom = " + Environment.NewLine;
                                for (int i = 0; i < NewSubject.AccelOfBlinks[0].Count; i++)
                                {
                                    TOPRINT += NewSubject.AccelOfBlinks[0][i][40].ToString() + Environment.NewLine;
                                }
                                TOPRINT += "Accel of Right MidTop = " + Environment.NewLine;
                                for (int i = 0; i < NewSubject.AccelOfBlinks[0].Count; i++)
                                {
                                    TOPRINT += NewSubject.AccelOfBlinks[0][i][17].ToString() + Environment.NewLine;
                                }
                                TOPRINT += "Accel of Right MidTBottom = " + Environment.NewLine;
                                for (int i = 0; i < NewSubject.AccelOfBlinks[0].Count; i++)
                                {
                                    TOPRINT += NewSubject.AccelOfBlinks[0][i][49].ToString() + Environment.NewLine;
                                }
                                NewSubject.AccelOfBlinks.Clear();
                            }
                            FrameBlink = 0;
                            BetweenBlink += 1;
                            TOPRINT += "Frame betweem blink, Frame: " + BetweenBlink.ToString() + Environment.NewLine;
                            if (LClosed == "Yes" || RClosed == "Yes")
                            {
                                TOPRINT += "Wink! not a blink!" + Environment.NewLine;
                            }
                            EndBlink = 0;
                        }
                    }
                }
                Writer.WriteLine(TOPRINT);
                TOPRINT = "";
            }
        }
        /////////////////////END OF FACE READER CODE/////////////////////

        /// <summary>
        /// Initializes a 3D mesh to deform every frame
        /// </summary>
        private void InitializeMesh()
        {
            Writer.WriteLine("----------------New Data Set-------------------" + Environment.NewLine);
            totalFrames = 0;
            var vertices = this.currentFaceModel.CalculateVerticesForAlignment(this.currentFaceAlignment);

            var triangleIndices = this.currentFaceModel.TriangleIndices;

            var indices = new Int32Collection(triangleIndices.Count);

            for (int i = 0; i < triangleIndices.Count; i += 3)
            {
                uint index01 = triangleIndices[i];
                uint index02 = triangleIndices[i + 1];
                uint index03 = triangleIndices[i + 2];

                indices.Add((int)index03);
                indices.Add((int)index02);
                indices.Add((int)index01);
            }

            this.theGeometry.TriangleIndices = indices;
            this.theGeometry.Normals = null;
            this.theGeometry.Positions = new Point3DCollection();
            this.theGeometry.TextureCoordinates = new PointCollection();

            for (int i = 0; i < vertices.Count; i++)
            {
                var vert = vertices[i];
                this.theGeometry.Positions.Add(new Point3D(vert.X, vert.Y, -vert.Z));
                this.theGeometry.TextureCoordinates.Add(new Point());
            }
            TOPRINT += "PRE CAPTURE FRAME" + Environment.NewLine;
            Writer.WriteLine(TOPRINT);
            TOPRINT = "";
        }

        /// <summary>
        /// Sends the new deformed mesh to be drawn
        /// </summary>
        private void UpdateMesh()
        {
            var vertices = this.currentFaceModel.CalculateVerticesForAlignment(this.currentFaceAlignment);
            TempList = new List<CameraSpacePoint>();
            TempList.Clear();
            for (int i = 0; i < vertices.Count; i++)
            {
                var vert = vertices[i];
                this.theGeometry.Positions[i] = new Point3D(vert.X, vert.Y, -vert.Z);
            }
            foreach (int Index in LEyeIndex)
            {
                TempList.Add(vertices[Index]);
            }
            foreach (int Index in REyeIndex)
            {
                TempList.Add(vertices[Index]);
            }
            PointsData.Add(TempList);
            if (PointsData.Count > 12)
            {
                PointsData.RemoveAt(0);
            }
            Writer.WriteLine(TOPRINT);
            TOPRINT = "";
        }

        /// <summary>
        /// Start a face capture on clicking the button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void StartCapture_Button_Click(object sender, RoutedEventArgs e)
        {
            this.StartCapture();
        }

        /// <summary>
        /// This event fires when a BodyFrame is ready for consumption
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            this.CheckOnBuilderStatus();

            var frameReference = e.FrameReference;
            using (var frame = frameReference.AcquireFrame())
            {
                if (frame == null)
                {
                    // We might miss the chance to acquire the frame, it will be null if it's missed
                    return;
                }

                if (this.currentTrackedBody != null)
                {
                    this.currentTrackedBody = FindBodyWithTrackingId(frame, this.CurrentTrackingId);

                    if (this.currentTrackedBody != null)
                    {
                        return;
                    }
                }

                Body selectedBody = FindClosestBody(frame);

                if (selectedBody == null)
                {
                    return;
                }

                this.currentTrackedBody = selectedBody;
                this.CurrentTrackingId = selectedBody.TrackingId;

                this.highDefinitionFaceFrameSource.TrackingId = this.CurrentTrackingId;
                this.faceFrameSource.TrackingId = this.currentTrackingId;
            }
        }

        /// <summary>
        /// This event is fired when a tracking is lost for a body tracked by HDFace Tracker
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void HdFaceSource_TrackingIdLost(object sender, TrackingIdLostEventArgs e)
        {
            var lostTrackingID = e.TrackingId;

            if (this.CurrentTrackingId == lostTrackingID)
            {
                this.CurrentTrackingId = 0;
                this.currentTrackedBody = null;
                if (this.faceModelBuilder != null)
                {
                    this.faceModelBuilder.Dispose();
                    this.faceModelBuilder = null;
                }

                this.highDefinitionFaceFrameSource.TrackingId = 0;
            }
        }

        /// <summary>
        /// This event is fired when a new HDFace frame is ready for consumption
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void HdFaceReader_FrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                // We might miss the chance to acquire the frame; it will be null if it's missed.
                // Also ignore this frame if face tracking failed.
                if (frame == null || !frame.IsFaceTracked)
                {
                    return;
                }

                frame.GetAndRefreshFaceAlignmentResult(this.currentFaceAlignment);
                this.UpdateMesh();
                if (frame != null)
                {
                    this.Clock.Add(frame.RelativeTime);
                }
                int sizeofClock;
                sizeofClock = this.Clock.Count;
                if (sizeofClock == 1)
                {
                    TOPRINT += "Frame " + totalFrames.ToString() + " NO DATA" + Environment.NewLine;
                }
                if (sizeofClock > 1)
                {
                    double dFramesDropped = ((Clock[1].TotalSeconds - Clock[0].TotalSeconds) / (0.0333333333)) - 1;
                    int FramesDropped = (int)dFramesDropped;
                    totalFrames += FramesDropped;
                    TOPRINT += "Frame " + totalFrames.ToString() + Environment.NewLine;

                    if (FramesDropped >= 1)
                    {
                        TOPRINT += "Frames Dropped: " + FramesDropped.ToString() + Environment.NewLine;
                    }
                    Clock.RemoveAt(0);
                }
                totalFrames += 1;
            }
        }

        /// <summary>
        /// Start a face capture operation
        /// </summary>
        private void StartCapture()
        {
            this.StopFaceCapture();

            this.faceModelBuilder = null;

            this.faceModelBuilder = this.highDefinitionFaceFrameSource.OpenModelBuilder(FaceModelBuilderAttributes.None);

            this.faceModelBuilder.BeginFaceDataCollection();

            this.faceModelBuilder.CollectionCompleted += this.HdFaceBuilder_CollectionCompleted;

        }

        /// <summary>
        /// Cancel the current face capture operation
        /// </summary>
        private void StopFaceCapture()
        {
            if (this.faceModelBuilder != null)
            {
                this.faceModelBuilder.Dispose();
                this.faceModelBuilder = null;
            }
        }

        /// <summary>
        /// This event fires when the face capture operation is completed
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void HdFaceBuilder_CollectionCompleted(object sender, FaceModelBuilderCollectionCompletedEventArgs e)
        {
            var modelData = e.ModelData;

            this.currentFaceModel = modelData.ProduceFaceModel();

            this.faceModelBuilder.Dispose();
            this.faceModelBuilder = null;

            this.CurrentBuilderStatus = "Capture Complete";
        }

        /// <summary>
        /// Check the face model builder status
        /// </summary>
        private void CheckOnBuilderStatus()
        {
            if (this.faceModelBuilder == null)
            {
                return;
            }

            string newStatus = string.Empty;

            var captureStatus = this.faceModelBuilder.CaptureStatus;
            newStatus += captureStatus.ToString();

            var collectionStatus = this.faceModelBuilder.CollectionStatus;

            newStatus += ", " + GetCollectionStatusText(collectionStatus);

            this.CurrentBuilderStatus = newStatus;
        }
        static void DAnalyze(BiometricData Unknown)
        {
            Process Analyze = new Process();
            Analyze.StartInfo.FileName = "cmd.exe";
            Analyze.StartInfo.Arguments = "/c DIR"; // Note the /c command (*)
            Analyze.StartInfo.UseShellExecute = false;
            Analyze.StartInfo.RedirectStandardOutput = true;
            Analyze.StartInfo.RedirectStandardError = true;
            Analyze.Start();
            //* Read the output (or the error)
            string output = Analyze.StandardOutput.ReadToEnd();
            Console.WriteLine(output);
            string err = Analyze.StandardError.ReadToEnd();
            Console.WriteLine(err);
            Analyze.WaitForExit();
        }
    }
}
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;


namespace DroneApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        bool auto=false;
        UDPExample UDPex;
        DispatcherTimer sendTimer = new DispatcherTimer();
        // Compose a string that consists of three lines.
        

        // Write the string to a file.
        //System.IO.StreamWriter file = new System.IO.StreamWriter("data.m");
        
        public MainWindow()
        {
            InitializeComponent();
            DispatcherTimer dispatcherTimer = new DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);
            dispatcherTimer.Start();

            
            sendTimer.Tick += new EventHandler(sendTimer_Tick);
            sendTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);
            

            UDPex = new UDPExample();
            slider_chan1.Value = 50;
            slider_chan3.Value = 50;
            label_chan1.Content = Math.Floor(slider_chan1.Value);
            label_chan2.Content = Math.Floor(slider_chan2.Value);
            label_chan3.Content = Math.Floor(slider_chan3.Value);
            label_chan4.Content = Math.Floor(slider_chan4.Value);
            label_thrust.Content = Math.Floor(slider_thrust.Value);
            if (auto)
                mode.Content = "Auto";
            else
                mode.Content = "Manual";

            double wxmin = -10;
            double wxmax = 110;
            double wymin = -1;
            double wymax = 11;
            const double xstep = 10;
            const double ystep = 1;

            const double dmargin = 10;
            double dxmin = dmargin;
            double dxmax = canGraph.Width - dmargin;
            double dymin = dmargin;
            double dymax = canGraph.Height - dmargin;

            // Prepare the transformation matrices.
            PrepareTransformations(
                wxmin, wxmax, wymin, wymax,
                dxmin, dxmax, dymax, dymin);

            // Get the tic mark lengths.
            Point p0 = DtoW(new Point(0, 0));
            Point p1 = DtoW(new Point(5, 5));
            double xtic = p1.X - p0.X;
            double ytic = p1.Y - p0.Y;

            // Make the X axis.
            GeometryGroup xaxis_geom = new GeometryGroup();
            p0 = new Point(wxmin, 0);
            p1 = new Point(wxmax, 0);
            xaxis_geom.Children.Add(new LineGeometry(WtoD(p0), WtoD(p1)));

            for (double x = xstep; x <= wxmax - xstep; x += xstep)
            {
                // Add the tic mark.
                Point tic0 = WtoD(new Point(x, -ytic));
                Point tic1 = WtoD(new Point(x, ytic));
                xaxis_geom.Children.Add(new LineGeometry(tic0, tic1));

                // Label the tic mark's X coordinate.
                DrawText(canGraph, x.ToString(),
                    new Point(tic0.X, tic0.Y + 5), 12,
                    HorizontalAlignment.Center,
                    VerticalAlignment.Top);
            }

            Path xaxis_path = new Path();
            xaxis_path.StrokeThickness = 1;
            xaxis_path.Stroke = Brushes.Black;
            xaxis_path.Data = xaxis_geom;

            canGraph.Children.Add(xaxis_path);

            // Make the Y axis.
            GeometryGroup yaxis_geom = new GeometryGroup();
            p0 = new Point(0, wymin);
            p1 = new Point(0, wymax);
            xaxis_geom.Children.Add(new LineGeometry(WtoD(p0), WtoD(p1)));

            for (double y = ystep; y <= wymax - ystep; y += ystep)
            {
                // Add the tic mark.
                Point tic0 = WtoD(new Point(-xtic, y));
                Point tic1 = WtoD(new Point(xtic, y));
                xaxis_geom.Children.Add(new LineGeometry(tic0, tic1));

                // Label the tic mark's Y coordinate.
                DrawText(canGraph, y.ToString(),
                    new Point(tic0.X - 10, tic0.Y), 12,
                    HorizontalAlignment.Center,
                    VerticalAlignment.Center);
            }

            Path yaxis_path = new Path();
            yaxis_path.StrokeThickness = 1;
            yaxis_path.Stroke = Brushes.Black;
            yaxis_path.Data = yaxis_geom;

            canGraph.Children.Add(yaxis_path);

            // Make some data sets.
            Brush[] brushes = { Brushes.Red, Brushes.Green, Brushes.Blue };
            Random rand = new Random();
            for (int data_set = 0; data_set < 3; data_set++)
            {
                double last_y = rand.Next(3, 7);

                PointCollection points = new PointCollection();
                for (double x = 0; x <= 100; x += 10)
                {
                    last_y += rand.Next(-10, 10) / 10.0;
                    if (last_y < 0) last_y = 0;
                    if (last_y > 10) last_y = 10;
                    Point p = new Point(x, last_y);
                    points.Add(WtoD(p));
                }

                Polyline polyline = new Polyline();
                polyline.StrokeThickness = 1;
                polyline.Stroke = brushes[data_set];
                polyline.Points = points;

                canGraph.Children.Add(polyline);
            }

            // Make a title
            Point title_location = WtoD(new Point(50, 10));
            DrawText(canGraph, "Amazing Data", title_location, 20,
                HorizontalAlignment.Center,
                VerticalAlignment.Top);
        
        }
        // Prepare values for perform transformations.
        private Matrix WtoDMatrix, DtoWMatrix;
        private void PrepareTransformations(
            double wxmin, double wxmax, double wymin, double wymax,
            double dxmin, double dxmax, double dymin, double dymax)
        {
            // Make WtoD.
            WtoDMatrix = Matrix.Identity;
            WtoDMatrix.Translate(-wxmin, -wymin);

            double xscale = (dxmax - dxmin) / (wxmax - wxmin);
            double yscale = (dymax - dymin) / (wymax - wymin);
            WtoDMatrix.Scale(xscale, yscale);

            WtoDMatrix.Translate(dxmin, dymin);

            // Make DtoW.
            DtoWMatrix = WtoDMatrix;
            DtoWMatrix.Invert();
        }

        // Transform a point from world to device coordinates.
        private Point WtoD(Point point)
        {
            return WtoDMatrix.Transform(point);
        }

        // Transform a point from device to world coordinates.
        private Point DtoW(Point point)
        {
            return DtoWMatrix.Transform(point);
        }

        // Position a label at the indicated point.
        private void DrawText(Canvas can, string text, Point location,
            double font_size,
            HorizontalAlignment halign, VerticalAlignment valign)
        {
            // Make the label.
            Label label = new Label();
            label.Content = text;
            label.FontSize = font_size;
            can.Children.Add(label);

            // Position the label.
            label.Measure(new Size(double.MaxValue, double.MaxValue));

            double x = location.X;
            if (halign == HorizontalAlignment.Center)
                x -= label.DesiredSize.Width / 2;
            else if (halign == HorizontalAlignment.Right)
                x -= label.DesiredSize.Width;
            Canvas.SetLeft(label, x);

            double y = location.Y;
            if (valign == VerticalAlignment.Center)
                y -= label.DesiredSize.Height / 2;
            else if (valign == VerticalAlignment.Bottom)
                y -= label.DesiredSize.Height;
            Canvas.SetTop(label, y);
        }
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            
        }
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            if (!string.IsNullOrEmpty(UDPExample.m_data))
            {
                string[] words = UDPExample.m_data.Split(';');
                string status = null;
                if (words[0] == "0")
                    status = "Invalid data";
                gps.Content = "Lattitude: " + words[1].ToString() + "\n" + "Longitude: " + words[2].ToString() + "\n" + "Height: " + words[3].ToString() + "\n" + status;
                mpu.Content = "X: " + words[4] + "\n" + "Y: " + words[5] + "\n";
                float voltage = (Convert.ToSingle(words[6]) / 1024.0f) * 5.2f * (102.0f / 27.0f);
                telemetry.Content = "Voltage: " + voltage + "\n" + "per cell: " + voltage / 3 + "\n" + "Pressure: " + words[7].ToString() + "\n" + "Temp: " + words[8].ToString();

            }




        }

        private void sendTimer_Tick(object sender, EventArgs e)
        {
            string binAuto;
            if (auto)
                binAuto = "1";
            else
                binAuto = "0";
            UDPExample.m_auto = binAuto + ";" + slider_chan1.Value.ToString() + ";" + slider_chan2.Value.ToString() + ";" + slider_chan3.Value.ToString() + ";" + slider_chan4.Value.ToString() + ";" + slider_thrust.Value.ToString() + "\n";
            UDPExample.SendData();

        }

        public static bool PingHost(string nameOrAddress)
        {
            bool pingable = false;
            Ping pinger = new Ping();
            try
            {
                PingReply reply = pinger.Send(nameOrAddress);
                pingable = reply.Status == IPStatus.Success;
            }
            catch (PingException)
            {
                // Discard PingExceptions and return false;
            }
            return pingable;
        }

        private void button_ping_Click(object sender, RoutedEventArgs e)
        {
            if (PingHost(textBox_ping.Text))
                label_ping.Content = "Ping successful";
            else
                label_ping.Content = "Cannot reach host";
        }

        private void slider_chan1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_chan1.Content = Math.Floor(slider_chan1.Value);
        }

        private void slider_chan2_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_chan2.Content = Math.Floor(slider_chan2.Value);
        }

        private void slider_chan3_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_chan3.Content = Math.Floor(slider_chan3.Value);
        }

        private void slider_chan4_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_chan4.Content = Math.Floor(slider_chan4.Value);
        }

        private void connect_Click(object sender, RoutedEventArgs e)
        {
            Thread recvThread = new Thread(new ThreadStart(UDPExample.Run));
            recvThread.IsBackground = true;
            recvThread.Start();
            sendTimer.Start();
        }

        private void mode_Click(object sender, RoutedEventArgs e)
        {
            auto = !auto;
            if (auto)
                mode.Content = "Auto";
            else
                mode.Content = "Manual";
            slider_chan1.IsEnabled = !auto;
            slider_chan2.IsEnabled = !auto;
            slider_chan3.IsEnabled = !auto;
            slider_chan4.IsEnabled = !auto;
            slider_thrust.IsEnabled = auto;
        }

        private void button_write_Click(object sender, RoutedEventArgs e)
        {
            //file.WriteLine(lines);

            //file.Close();
        }

        private void slider_thrust_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_thrust.Content = Math.Floor(slider_thrust.Value);
        }
    }

    public class UDPExample
    {
        public static string m_data;
        public static string m_auto;

        public static void Run()
        {

            //naslouchani spustime ve vlastnim vlakne
            Thread listener = new Thread(new ThreadStart(DoListening));
            listener.IsBackground = true;
            listener.Start();
            //pockame vterinu a zasleme data
            Thread.Sleep(3000);

            //SendData();
        }

        static void DoListening()
        {

            UdpClient client = null;
            try
            {
                //zacneme poslouchat na portu 3000
                client = new UdpClient(8888);
                //budeme primat z jakekoli adresy a portu
                IPEndPoint host = new IPEndPoint(IPAddress.Any, 0);
                Console.WriteLine("{0} : UDP posluchac ceka na data..", DateTime.Now.ToString());

                //pockame na prijeti dat
                while (true)
                {
                    byte[] received = client.Receive(ref host);
                    //ziskame z bajtu retezec
                    string receivedString = Encoding.ASCII.GetString(received);
                    m_data = receivedString;

                    Console.WriteLine("{0} : UDP posluchac prijal od {1} data : {2}", DateTime.Now.ToString(), host.ToString(), receivedString);
                }
            }
            catch (SocketException ex)
            {

                Console.WriteLine("Doslo k vyjimce z duvodu : {0}", ex.SocketErrorCode);
            }
            finally
            {
                if (client != null)
                {
                    client.Close();
                }
            }
        }

        public static void SendData()
        {
            UdpClient client = null;
            try
            {
                client = new UdpClient();
                //pripojime se na cil, kteremu chceme zaslat data
                client.Connect("192.168.1.100", 9999);
                //enkodujeme retezec a zasleme na cil
                byte[] data = Encoding.ASCII.GetBytes(m_auto + " ");
                client.Send(data, data.Length);
                Console.WriteLine("{0} : UDP vysilac vyslal data : {1}", DateTime.Now.ToString(), m_auto);
            }
            catch (SocketException ex)
            {
                Console.WriteLine("Doslo k vyjimce z duvodu : {0}", ex.SocketErrorCode);
            }
            finally
            {
                if (client != null)
                {
                    client.Close();
                }
            }
        }

    }
}

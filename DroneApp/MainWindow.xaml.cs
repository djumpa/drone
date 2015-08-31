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

        public MainWindow()
        {
            InitializeComponent();
            DispatcherTimer dispatcherTimer = new DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);
            dispatcherTimer.Start();
            UDPex = new UDPExample();
            label_chan1.Content = Math.Floor(slider_chan1.Value);
            label_chan2.Content = Math.Floor(slider_chan2.Value);
            label_chan3.Content = Math.Floor(slider_chan3.Value);
            label_chan4.Content = Math.Floor(slider_chan4.Value);

            if (auto)
                mode.Content = "Auto";
            else
                mode.Content = "Manual";

            const double margin = 10;
            double xmin = margin;
            double xmax = canGraph.Width - margin;
            double ymin = margin;
            double ymax = canGraph.Height - margin;
            const double step = 10;

            // Make the X axis.
            GeometryGroup xaxis_geom = new GeometryGroup();
            xaxis_geom.Children.Add(new LineGeometry(
                new Point(0, ymax), new Point(canGraph.Width, ymax)));
            for (double x = xmin + step;
                x <= canGraph.Width - step; x += step)
            {
                xaxis_geom.Children.Add(new LineGeometry(
                    new Point(x, ymax - margin / 2),
                    new Point(x, ymax + margin / 2)));
            }

            Path xaxis_path = new Path();
            xaxis_path.StrokeThickness = 1;
            xaxis_path.Stroke = Brushes.Black;
            xaxis_path.Data = xaxis_geom;

            canGraph.Children.Add(xaxis_path);

            // Make the Y ayis.
            GeometryGroup yaxis_geom = new GeometryGroup();
            yaxis_geom.Children.Add(new LineGeometry(
                new Point(xmin, 0), new Point(xmin, canGraph.Height)));
            for (double y = step; y <= canGraph.Height - step; y += step)
            {
                yaxis_geom.Children.Add(new LineGeometry(
                    new Point(xmin - margin / 2, y),
                    new Point(xmin + margin / 2, y)));
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
                int last_y = rand.Next((int)ymin, (int)ymax);

                PointCollection points = new PointCollection();
                for (double x = xmin; x <= xmax; x += step)
                {
                    last_y = rand.Next(last_y - 10, last_y + 10);
                    if (last_y < ymin) last_y = (int)ymin;
                    if (last_y > ymax) last_y = (int)ymax;
                    points.Add(new Point(x, last_y));
                }

                Polyline polyline = new Polyline();
                polyline.StrokeThickness = 1;
                polyline.Stroke = brushes[data_set];
                polyline.Points = points;

                canGraph.Children.Add(polyline);
            }
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

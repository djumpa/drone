using System;
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
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace DroneApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DispatcherTimer dispatcherTimer = new DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 0, 20);
            dispatcherTimer.Start();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            
        }
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            
            
            
            

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
    }

    
}

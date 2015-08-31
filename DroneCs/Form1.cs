using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;


namespace DroneCs
{
    public partial class Form1 : Form
    {

        UDPExample UDPex;
        bool auto;

        public Form1()
        {
            InitializeComponent();
            UDPex = new UDPExample();
            label5.Text = trackBar1.Value.ToString();
            label6.Text = trackBar2.Value.ToString();
            label7.Text = trackBar3.Value.ToString();
            label8.Text = trackBar4.Value.ToString();
            label12.Text = trackBar5.Value.ToString();
            if (auto)
                button3.Text = "Auto";
            else
                button3.Text = "Manual";
        }
        

        
        private void button1_Click(object sender, EventArgs e)
        {
            if(!backgroundWorker1.IsBusy)
                backgroundWorker1.RunWorkerAsync();

            sendTimer.Enabled = !sendTimer.Enabled;

        }

        private void Form1_Load(object sender, EventArgs e)
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


        private void button2_Click(object sender, EventArgs e)
        {
            //label1.Text = textBox1.Text;
            if (PingHost(textBox1.Text))
                label1.Text = "Ping successful";
            else
                label1.Text = "Cannot reach host";
        }

        private void label3_Click(object sender, EventArgs e)
        {

        }

        private void backgroundWorker1_DoWork(object sender, DoWorkEventArgs e)
        {
            UDPExample.Run();
           
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        

        private void textBox2_TextChanged(object sender, EventArgs e)
        {

        }

        private void label4_Click(object sender, EventArgs e)
        {

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void refresh_Tick(object sender, EventArgs e)
        {

 
            if (!string.IsNullOrEmpty(UDPExample.m_data))
            {
                string[] words = UDPExample.m_data.Split(';');
                string status = null;
                if (words[0] == "0")
                    status = "Invalid data";
                label_GPS.Text ="Lattitude: " + words[1].ToString() + "\n" + "Longitude: " + words[2].ToString() + "\n" + "Height: " + words[3].ToString() + "\n" + status;
                label_Acc.Text = "X: " + words[4] + "\n" + "Y: " + words[5] + "\n";
                float voltage = (Convert.ToSingle(words[6])/1024.0f)*5.2f*(102.0f/27.0f);
                label_telemetry.Text = "Voltage: " + voltage +"\n" + "per cell: "+voltage/3+ "\n" + "Pressure: " + words[7].ToString() + "\n" + "Temp: " + words[8].ToString();
                
            }

        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            label5.Text = trackBar1.Value.ToString();
        }

        private void trackBar2_Scroll(object sender, EventArgs e)
        {
            label6.Text = trackBar2.Value.ToString();
        }

        private void trackBar3_Scroll(object sender, EventArgs e)
        {
            label7.Text = trackBar3.Value.ToString();
        }

        private void trackBar4_Scroll(object sender, EventArgs e)
        {
            label8.Text = trackBar4.Value.ToString();
        }

        private void label5_Click(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)
        {
            auto = !auto;
            if (auto)
                button3.Text = "Auto";
            else
                button3.Text = "Manual";
            trackBar1.Enabled = !auto;
            trackBar2.Enabled = !auto;
            trackBar3.Enabled = !auto;
            trackBar4.Enabled = !auto;
            
        }

        private void backgroundWorker1_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            if (e.Cancelled) MessageBox.Show("Operation was canceled");
            else if (e.Error != null) MessageBox.Show(e.Error.Message);
            else MessageBox.Show(e.Result.ToString());
        }

        private void sendTimer_Tick(object sender, EventArgs e)
        {
            string binAuto;
            if (auto)
                binAuto = "1";
            else
                binAuto = "0";
            UDPExample.m_auto = binAuto + ";" + trackBar1.Value.ToString() + ";" + trackBar2.Value.ToString() + ";" + trackBar3.Value.ToString() + ";" + trackBar4.Value.ToString() + ";"  +trackBar5.Value.ToString() + "\n";
            UDPExample.SendData();
        }

        private void telemetry_Click(object sender, EventArgs e)
        {

        }

        private void label3_Click_1(object sender, EventArgs e)
        {

        }

        private void trackBar5_Scroll(object sender, EventArgs e)
        {
            label12.Text = trackBar5.Value.ToString();
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
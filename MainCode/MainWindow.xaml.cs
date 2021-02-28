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

namespace MainCode
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        int HLow = 0;
        public object NavigationService { get; private set; }
        public ImageControlPage ImgControlPage;

        public MainWindow()
        {
            InitializeComponent();
        }
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            MainFrame.Navigate(new ImageControlPage());
        }

        private void IntroductionButton_Checked(object sender, RoutedEventArgs e)
        {
            MainFrame.Navigate(new IntroductionPage());
        }

        private void ImageControlButton_Checked(object sender, RoutedEventArgs e)
        {
            MainFrame.Navigate(new ImageControlPage());
        }

        private void MainControlButton_Checked(object sender, RoutedEventArgs e)
        {
            ImgControlPage = new ImageControlPage();
            ImgControlPage.ProduceThresholdImage();
            HLow = ImgControlPage.HueLow;
        }


    }
}

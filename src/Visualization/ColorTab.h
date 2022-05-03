#ifndef COLOR_TAB_H
#define COLOR_TAB_H
namespace one_piece
{
namespace visualization
{
    //color table from scannet
    
    static std::vector<cv::Scalar> color_tab =   
    {
             cv::Scalar(255, 255, 255),
             cv::Scalar(174, 199, 232),     // wall
             cv::Scalar(152, 223, 138),     // floor
             cv::Scalar(31, 119, 180),      // cabinet
             cv::Scalar(255, 187, 120),     // bed
             cv::Scalar(188, 189, 34),      // chair
             cv::Scalar(140, 86, 75),       // sofa
             cv::Scalar(255, 152, 150),     // table
             cv::Scalar(214, 39, 40),       // door
             cv::Scalar(197, 176, 213),     // window
             cv::Scalar(148, 103, 189),     // bookshelf
             cv::Scalar(196, 156, 148),     // picture
             cv::Scalar(23, 190, 207),      // counter
             cv::Scalar(178, 76, 76),
             cv::Scalar(247, 182, 210),     // desk
             cv::Scalar(66, 188, 102),
             cv::Scalar(219, 219, 141),     // curtain
             cv::Scalar(140, 57, 197),
             cv::Scalar(202, 185, 52),
             cv::Scalar(51, 176, 203),
             cv::Scalar(200, 54, 131),
             cv::Scalar(92, 193, 61),
             cv::Scalar(78, 71, 183),
             cv::Scalar(172, 114, 82),
             cv::Scalar(255, 127, 14),      // refrigerator
             cv::Scalar(91, 163, 138),
             cv::Scalar(153, 98, 156),
             cv::Scalar(140, 153, 101),
             cv::Scalar(158, 218, 229),     // shower curtain
             cv::Scalar(100, 125, 154),
             cv::Scalar(178, 127, 135),
             cv::Scalar(120, 185, 128),
             cv::Scalar(146, 111, 194),
             cv::Scalar(44, 160, 44),       // toilet
             cv::Scalar(112, 128, 144),     // sink
             cv::Scalar(96, 207, 209),
             cv::Scalar(227, 119, 194),     // bathtub
             cv::Scalar(213, 92, 176),
             cv::Scalar(94, 106, 211),
             cv::Scalar(82, 84, 163),       // otherfurn
             cv::Scalar(100, 85, 144)
    };    
}
}

#endif
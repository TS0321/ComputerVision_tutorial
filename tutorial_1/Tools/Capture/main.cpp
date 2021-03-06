#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
int main(int argh, char* argv[])
{
    cv::VideoCapture cap(1);//�f�o�C�X�̃I�[�v��
    //cap.open(0);//�������ł��ǂ��D
    //cap.set(cv::CAP_PROP_FOCUS, 500);
    const int camera_width = 1280;
    const int camera_height = 720;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);

    if (!cap.isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
    {
        //�ǂݍ��݂Ɏ��s�����Ƃ��̏���
        return -1;
    }

    cv::Mat frame; //�擾�����t���[��
    int img_num = 0;
    while (cap.read(frame))//�������[�v
    {
        //
        //�擾�����t���[���摜�ɑ΂��āC�N���[�X�P�[���ϊ���2�l���Ȃǂ̏������������ށD
        //

        cv::imshow("win", frame);//�摜��\���D
        const int key = cv::waitKey(1);
        if (key == 'q'/*113*/)//q�{�^���������ꂽ�Ƃ�
        {
            break;//while���[�v���甲����D
        }
        else if (key == 's'/*115*/)//s�������ꂽ�Ƃ�
        {
            //�t���[���摜��ۑ�����D
            std::string filename;
            filename = "../../../Capdata/img" + std::to_string(img_num++) + ".png";
            cv::imwrite(filename, frame);
            cv::waitKey(10);
            std::cout << filename << " is saved!" << std::endl;
        }
    }
    cv::destroyAllWindows();
    return 0;
}
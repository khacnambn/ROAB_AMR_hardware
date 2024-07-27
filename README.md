# ROAB_lab_MobileRobot
I. Tổng quan

1. Môi trường platform.ini (Khi nhấn Save file ini platformIO sẽ thực hiện build code)
   File bao gồm: + Định nghĩa phần cứng và các cổng giao tiếp
                 + lib_deps: các thư viện (có sẵn) cần cài
                 + Những dependencies liên quan ..
2. Các thư viện riêng sẽ lưu trong các file header trong Folder "lib"
3. Định nghĩa các chân, các thông số constant dùng trong firmware sẽ được lưu trong Folder "config"

II. Chạy firmware 
Test
1. Nap code lên ESP32S3 Chú ý thứ tự cắm chân sẽ là thứ tự cổng nạp và đọc data (Cổng gần đèn Leb cắm vào sau ACM1, cổng còn lại cắm vào trước ACM0)
2. Chạy micro ROS trên terminal: 
   Terminal 1: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1
   Terminal 2: ros2 run teleop_twist_keyboard teleop_twist_keyboard  (điều khiển động cơ)
   Terminal 3: ros2 topic list (kiểm tra các topic)
    /cmd_vel
    /imu/data
    /odom/unfiltered
    /parameter_events
    /rosout

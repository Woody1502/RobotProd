sudo cp /home/vim/RobotProd/robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service

# проверить статус
sudo systemctl status robot.service

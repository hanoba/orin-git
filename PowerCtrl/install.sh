sudo systemctl stop PowerCtrl.service
sudo cp /home/harald/orin-git/PowerCtrl/PowerCtrl.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable PowerCtrl.service
sudo systemctl start PowerCtrl.service
sudo systemctl status PowerCtrl.service

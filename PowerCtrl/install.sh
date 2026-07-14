sudo systemctl stop RaspiTriggerShutdown.service
sudo cp RaspiTriggerShutdown.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable RaspiTriggerShutdown.service
sudo systemctl start RaspiTriggerShutdown.service
sudo systemctl status RaspiTriggerShutdown.service

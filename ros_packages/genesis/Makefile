install:
	chmod +x genesis.bash
	cp genesis.service /usr/lib/systemd/system
	systemctl daemon-reload

uninstall:
	rm /usr/lib/systemd/system/genesis.service
	systemctl daemon-reload


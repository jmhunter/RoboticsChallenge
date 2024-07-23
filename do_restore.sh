#!/bin/bash

if [ "$1" == "" ]; then
	TARGET=master1
else
 	TARGET=$1
fi

cat <<__EOF__

About to restore entire laptop from ${TARGET} image (hosted online)

You will need to enter the local password for the robot user, when prompted
by sudo below.

__EOF__

if [ "$TARGET" == "master1" ]; then
	sudo RSYNC_PASSWORD=roboteer325 \
		rsync -avxPz --delete \
		--exclude=lost+found/ \
		robot-restore@robot01.ninja.org.uk::restore_${TARGET}_boot/ \
		/boot/
elif [ "$TARGET" == "master2" ]; then
	sudo RSYNC_PASSWORD=roboteer325 \
		rsync -avxPz --delete \
		--exclude=lost+found/ \
		robot-restore@robot01.ninja.org.uk::restore_${TARGET}_boot_efi/ \
		/boot/efi/
else
	read -p "Unknown target $TARGET"
	exit 1
fi
sudo RSYNC_PASSWORD=roboteer325 \
	rsync -avxPz --delete \
	--exclude=lost+found/ \
	robot-restore@robot01.ninja.org.uk::restore_${TARGET}_/ \
	/
retval=$?

if [ $retval -eq 0 ]; then
        read -p "Successfully restored. Press ENTER to reboot" REPLY
	sudo shutdown -r now
else
        read -p "Failed to restore due to one or more errors. Press ENTER to close, then please try again" REPLY
fi


#!/bin/bash

if [ -d /opt/tools/arm-bcm2708 ]; then	#si le repertoire arm-linux-gnueabihf n existe pas

	if [ -d /usr/share/doc/python-pip ]; then  #si le repertoire python-pip exite

		if [ ! -d /usr/local/lib/python2.7/dist-packages/future ]; then #si le repertoire future n existe pas
			echo "installation du paquet python future"
			sudo pip install future

		else #si tous les repertoire sont installes
			echo "tous les répertoires sont installés : prêt à la compilation"
		fi

	else #si le repertoire python-pip n exite pas
			echo "installation du paquet python future et de python-pip"
			sudo apt-get install python-pip
			sudo pip install future
	fi

else	#si le repertoire arm-linux-gnueabihf n existe pas

	if [ -d /usr/share/doc/python-pip ]; then #si le repertoire python-pip exite

		if [ ! -d /usr/local/lib/python2.7/dist-packages/future ]; then #si le repertoire future n existe pas
			echo "installation du paquet python-pip et des outils aidant à la compilation pour la RPi"
			sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools
			sudo pip install future

		else 
			echo "installation des outils aidant à la compilation pour la RPi"
			sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools
		fi

	else #si rien n est installe
			echo "installation du paquet python future et de python-pip, ainsi que des outils aidant à la compilation pour la RPi"
			sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools
			sudo apt-get install python-pip
			sudo pip install future
		fi

fi
echo ""

if [ ! -d /usr/share/doc/expect ]; then #si expect n est pas installe (important pour entrer le mdp pour le drone)
	sudo apt-get install expect
fi

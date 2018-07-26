#!/bin/bash
if [ $# -gt 2 ];then #si il y a plus de quatre arguments
	echo "Trop d'argument"
	echo "usage : ./compilation.sh ou ./compilation.sh upload ou ./compilation.sh upload file ou encore ./compilation.sh upload file <fichier_parametre.conf>"

else
	chmod +x .check_fichiers.sh 
	./.check_fichiers.sh
	

	if [ $# -eq 2 ];then #si il y a deux arguments
		#envoi du fichier ParametresDrone.conf (on peut choisir un autre nom mais il faut modifier le programme, ce .sh ainsi que .envoi_fichier.sh et aussi le script MatLab)
		if [ $1 = 'upload' ] && [ $2 = 'file' ]; then #si les arguments sont upload et file

			if [ -e ../ParametresDrone.conf ]; then #si le fichier ParametresDrone.conf existe bien dans ardupilot
				# commande de compilation et d envoi du fichier pour le drone
				export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
				cd .. # retour au repertoire ardupilot
				waf="$PWD/modules/waf/waf-light" #on donne un nom plus court au compilateur
				./waf configure --board=navio2	#on configure pour le flight controler Navio2
				echo "Compilation du fichier"
				./waf --targets bin/arducopter #compilation dans le repertoire build/navio2/bin/
				echo ""
				cd compilation
				chmod +x .kill_process.sh #on rend le fichier executable
				chmod +x .reboot_drone.sh #on rend le fichier executable
				#on kill le processus arducopter qui tourne en arriere plan et on redemarre le drone apres les executions qui viennent juste apres
				#on ouvre sur un autre terminal
				xterm -e "echo 'Kill du process arducopter qui tourne sur le RPi';./.kill_process.sh;sleep 3;./.reboot_drone.sh " & 
				sleep 2
				echo "Envoi des fichiers vers la RPi"
				chmod +x .envoi_drone.sh #on rend le fichier executable
				./.envoi_drone.sh #on envoie le fichier au drone arducopter pendant que le fichier executer en arriere plan ne tourne plus et avant que le drone redemarre
				chmod +x .envoi_fichier.sh #on rend le fichier executable
				./.envoi_fichier.sh #on envoie le fichier parametre au drone pendant que le fichier executer en arriere plan ne tourne plus et avant que le drone redemarre
				sleep 4
				pkill xterm #on ferme l autre terminal

				echo "Compilation et envoi du fichier terminés"
				echo "Le drone va redémarrer"
			else
				chemin="pwd/.."
				echo "Fichier ParametresDrone.conf inexistant dans le répertoire $chemin"
			fi

			
		else 
			echo "seuls arguments acceptés : upload file"
		fi

	elif [ $# -eq 1 ];then #si l argument 1 existe
		if [ $1 = 'upload' ]; then
			# commande de compilation et d envoi du fichier pour le drone
			export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
			cd .. # retour au repertoire ardupilot
			waf="$PWD/modules/waf/waf-light" #on donne un nom plus court au compilateur
			./waf configure --board=navio2	#on configure pour le flight controler Navio2
			echo "Compilation du fichier"
			./waf --targets bin/arducopter #compilation dans le repertoire build/navio2/bin/
			echo ""
			cd compilation
			chmod +x .kill_process.sh #on rend le fichier executable
			chmod +x .reboot_drone.sh #on rend le fichier executable
			#on kill le processus arducopter qui tourne en arriere plan et on redemarre le drone apres les executions qui viennent juste apres
			#on ouvre sur un autre terminal
			xterm -e "echo 'Kill du process arducopter qui tourne sur le RPi';./.kill_process.sh;sleep 7;./.reboot_drone.sh " & 
			sleep 5
			echo "Envoi du fichier vers la RPi"
			chmod +x .envoi_drone.sh #on rend le fichier executable
			./.envoi_drone.sh #on envoie le fichier au drone pendant que le fichier executer en arriere plan ne tourne plus et avant que le drone redemarre
			sleep 3
			pkill xterm #on ferme l autre terminal

			echo "Compilation et envoi du fichier terminés"
			echo "Le drone va redémarrer"
		else 
			echo "seul argument accepté : upload"
		fi
	
	else #sinon on veut simplement compiler
		# commande de compilation et d envoi du fichier pour le drone
		export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
		cd .. # retour au repertoire ardupilot
		waf="$PWD/modules/waf/waf-light" #on donne un nom plus court au compilateur
		./waf configure --board=navio2	#on configure pour le flight controler Navio2
		echo "Compilation du fichier"
		./waf --targets bin/arducopter #compilation dans le repertoire build/navio2/bin/	
	fi
		
fi

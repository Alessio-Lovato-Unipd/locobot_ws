#!/bin/bash

echo "Installing Azure kinect suit."

# Azure Kinect
    tmp_folder=".tmp_kinect_setup"
    mkdir -p $tmp_folder || true

    # Get Azure Kinect libraries
    wget http://ftp.de.debian.org/debian/pool/main/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb -P $tmp_folder
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb -P $tmp_folder
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb -P $tmp_folder
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb -P $tmp_folder
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb -P $tmp_folder
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb -P $tmp_folder

    # Install Azure Kinect libraries
    sudo dpkg -i $tmp_folder/libsoundio1_1.1.0-1_amd64.deb
    sudo dpkg -i $tmp_folder/k4a-tools_1.4.1_amd64.deb
    sudo dpkg -i $tmp_folder/libk4a1.4_1.4.1_amd64.deb
    sudo dpkg -i $tmp_folder/libk4a1.4-dev_1.4.1_amd64.deb
    sudo dpkg -i $tmp_folder/libk4abt1.1_1.1.2_amd64.deb
    sudo dpkg -i $tmp_folder/libk4abt1.1-dev_1.1.2_amd64.deb

    # Set K4A rules
    k4a_rules_path="/etc/udev/rules.d/99-k4a.rules"
    if [ ! -f $k4a_rules_path ]; then
        sudo touch $k4a_rules_path

        echo "# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub" | sudo tee -a $k4a_rules_path
        echo "# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub" | sudo tee -a $k4a_rules_path
        echo "# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera" | sudo tee -a $k4a_rules_path
        echo "# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera" | sudo tee -a $k4a_rules_path
        echo "# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array" | sudo tee -a $k4a_rules_path
        echo "" | sudo tee -a $k4a_rules_path
        echo "BUS!=\"usb\", ACTION!=\"add\", SUBSYSTEM!==\"usb_device\", GOTO=\"k4a_logic_rules_end\"" | sudo tee -a $k4a_rules_path
        echo "" | sudo tee -a $k4a_rules_path
        echo "ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097a\", MODE=\"0666\", GROUP=\"plugdev\"" | sudo tee -a $k4a_rules_path
        echo "ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097b\", MODE=\"0666\", GROUP=\"plugdev\"" | sudo tee -a $k4a_rules_path
        echo "ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097c\", MODE=\"0666\", GROUP=\"plugdev\"" | sudo tee -a $k4a_rules_path
        echo "ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097d\", MODE=\"0666\", GROUP=\"plugdev\"" | sudo tee -a $k4a_rules_path
        echo "ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097e\", MODE=\"0666\", GROUP=\"plugdev\"" | sudo tee -a $k4a_rules_path
        echo "" | sudo tee -a $k4a_rules_path
        echo "LABEL=\"k4a_logic_rules_end\"" | sudo tee -a $k4a_rules_path
    fi

    rm -rf $tmp_folder
    
echo "Azure Kinect suite installed."





exit 0

# Set gateway to your PC

sudo ip route add default via 192.168.123.200

# Set DNS

echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf

Test it on the robot with:

ping -c 2 8.8.8.8        # test connectivity

ping -c 2 google.com      # test DNS

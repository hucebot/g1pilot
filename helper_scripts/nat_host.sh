# 1. Enable IP forwarding

sudo sysctl -w net.ipv4.ip_forward=1

# 2. NAT traffic from the robot subnet out through eno1

sudo iptables -t nat -A POSTROUTING -s 192.168.123.0/24 -o eno1 -j MASQUERADE

sudo iptables -A FORWARD -i eno2 -o eno1 -j ACCEPT

sudo iptables -A FORWARD -i eno1 -o eno2 -m state --state RELATED,ESTABLISHED -j ACCEPT

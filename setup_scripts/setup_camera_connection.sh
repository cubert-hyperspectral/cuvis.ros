#!/bin/bash
sudo nmcli connection modify "Wired connection 1" ipv4.method manual ipv4.address 10.10.10.10/24 ipv4.gateway 10.10.10.1 ipv4.dns 10.10.10.100
sudo nmcli connection up "Wired connection 1"
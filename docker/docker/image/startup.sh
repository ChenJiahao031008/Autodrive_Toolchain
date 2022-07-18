#!/bin/bash

mkdir -p /var/run/sshd

chown -R root:root /root

# /usr/bin/supervisord -c /etc/supervisor/supervisord.conf

launch all:
exec /usr/bin/tini -- /usr/bin/supervisord -n

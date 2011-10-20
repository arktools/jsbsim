#!/usr/bin/python
# Author: Lenna X. Peterson (github.com/lennax)

import smtplib # for sending email
from email.mime.text import MIMEText # for formatting message
import getpass # for getting password

## Connect to server ##
def connect_gmail():
	server = smtplib.SMTP('smtp.gmail.com', 587)
	server.ehlo()
	server.starttls()
	server.ehlo()
	return server

## Check login credentials ##
def authenticate(server, sender, password, close=False):
	if password == "":
		password = getpass.getpass("Enter password for '%s': " % sender)
	if close:
		print "Checking authentication"
	not_auth = True
	while (not_auth):
		try:
			server.login(sender,password)	
		except smtplib.SMTPAuthenticationError:
			print "Authentification failed, try again: "
			if sender != "arktools.github@gmail.com":
				sender = raw_input("Enter Gmail username: ")
				if "@" not in sender:
					sender += "@gmail.com"
			password = getpass.getpass("Enter password for '%s': " % sender)
			continue
		not_auth = False
	print "Logged in"
	if close:
		server.close()
		print "Closed connection"
	
## Send via gmail ##
# To call without specifying password, use ""
def send_gmail(sender, password, recipient, subject, message):
	msg = MIMEText(message)
	if password == "":
		password = getpass.getpass("Enter password for '%s': " % sender)
	msg['Subject'] = subject
	msg['From'] = sender 
	msg['To'] = recipient 
	print "Connecting to server"
	server = connect_gmail()
	print "Logging in"
	authenticate(server, sender, password)
	server.sendmail(sender, [recipient], msg.as_string())
	server.close()
	print "Sent mail"
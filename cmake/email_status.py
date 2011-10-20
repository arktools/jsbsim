#!/usr/bin/python
# Author: Lenna X. Peterson (github.com/lennax)
# 
# This script waits while a program runs, then emails exit status and runtime.
# Requires 'send_gmail.py'
# USAGE:
# It is designed to prefix a normal argument:
#   $ rm -f => $ ./script.py rm -f
# The script's options do not conflict with the called program's options: 
#   $ rm -f => $ ./script.py -f you@gmail.com rm -f
# But quoting is acceptable as well:
#   $ ./script.py "rm -rf"
#
# Then enter password and wait until you see "Running ..."

import optparse # for parsing options
import subprocess # for calling program 
import datetime # for timing program
import getpass # for getting password
try:
	import send_gmail # my module for sending
except ImportError:
	print "Could not find 'send_gmail.py'"
	print "This module is required."
	raise SystemExit

## Define defaults ##
SENDER = "arktools.github@gmail.com"
RECIPIENT = "arktools.github@gmail.com"

## Option parsing ## 
usage = "usage: %prog [options] program_to_run [program options]"
parser = optparse.OptionParser(usage=usage)
parser.disable_interspersed_args()
parser.set_defaults(sender=SENDER, recipient=RECIPIENT)
descript = optparse.OptionGroup(parser, "Description", """This script waits while a program runs, then emails the exit status and runtime.""")
parser.add_option_group(descript)
parser.add_option("-f", "--from", dest="sender",
		help="""Email to send from (must be a gmail address/alias, must know password, default %s)""" % SENDER)
parser.add_option("-t", "--to", dest="recipient",
		help="""Email to send to (default %s""" % RECIPIENT)
(options, args) = parser.parse_args()
if len(args) == 0:
	parser.print_usage()
	print "A program must be specified"
	raise SystemExit
elif len(args) == 1 and " " in args[0]:
	script = args[0].split(" ")
else:
	script = args
if "@" not in options.sender:
	options.sender += "@gmail.com"
if "@" not in options.recipient:
	options.recipient += "@gmail.com"
	print "Specified To: address did not include '@'"
	print "Trying %s@gmail.com" % options.recipient

## Verify credentials BEFORE running (long) process
pw = getpass.getpass("Enter password for '%s': " % options.sender)
server = send_gmail.connect_gmail()
send_gmail.authenticate(server, options.sender, pw, close=True)

## Notify process exit status by email ##
def status_email(script, sender, recipient):
	start = datetime.datetime.now()
	script_str = " ".join(script)
	print "Running '%s'..." % script_str
	status = subprocess.check_call(script)
	runtime = str(datetime.datetime.now() - start)
	message = "'%s' exited with status %s after %s" % (script_str, status, runtime)
	if status == 0:
		message = "Good news! " + message
	subject = "'%s' finished" % script_str
	print message
	print "Preparing to send mail"
	send_gmail.send_gmail(sender, pw, recipient, subject, message)

status_email(script, options.sender, options.recipient)

#! /usr/bin/env python3

# This script is for posting notification about experiments on Slack channel

import os
import sys
import requests


def post_message(message):
    webhook_url = os.environ['SLACK_WEBHOOK_URL']
    try:
        response = requests.post(
            webhook_url, json={'text': message},
            headers={'Content-Type': 'application/json'}
        )
    except ValueError:
        pass


def post_status_message(exp_exit_code, exp_num):
    positive_msg = 'Experiment{}{} completed successfully! :bb8flame:'
    negative_msg = ':alert: I have a bad feeling about experiment {}! :han-solo:'

    if exp_exit_code == 0:
        if exp_num > 0:
            message = positive_msg.format(' ', exp_num)
        else:
            message = positive_msg.format('s', '')
    else:
        message = negative_msg.format(exp_num)

    post_message(message)


if __name__ == '__main__':
    if len(sys.argv) == 2:
        post_message(str(sys.argv[1]))
    elif len(sys.argv) == 3:
        post_status_message(int(sys.argv[1]), sys.argv[2])




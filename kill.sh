#!/bin/bash

kill -9 $(ps | grep pytho[n] | awk {'print $1'})


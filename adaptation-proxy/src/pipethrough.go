package main

import (
    "io"
    "os"
    "os/exec"
    "log"
)

func CommandInPath(name string) bool {
	_, err := exec.LookPath(name)
	return err == nil
}

func PipeThrough(w io.Writer, r io.Reader, extraEnv []string, cmd string, cmdargs ...string) bool {
	log.Println("starting command", cmd)
	c2 := exec.Command(cmd, cmdargs...)

	c2.Stdin = r
	c2.Stdout = w
	c2.Stderr = os.Stderr

	currentEnv := os.Environ()
	c2.Env = make([]string, len(currentEnv) + len(extraEnv))
	copy(c2.Env, extraEnv)
	copy(c2.Env[len(extraEnv):], currentEnv)
	// copy(c2.Env, currentEnv)
	// copy(c2.Env[len(currentEnv):], extraEnv)
	log.Println("extraenv first: ", extraEnv[0])
	err := c2.Start()
	if err != nil {
		log.Println("failed to start command")
		return false
	}
	err = c2.Wait()
	if err != nil {
		log.Println("error exit from command:", err)
	}
	return true
}

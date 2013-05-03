package main

import (
	"code.google.com/p/go-icap"
	"log"
	"net/http"
	"io"
	"time"
	"strings"
	"regexp"
)

var ISTag = "\"picworsen\""

func main() {
	icap.ListenAndServe(":11344", icap.HandlerFunc(handleIcap))
}

// (lifted from redwood)
// copyResponseHeader writes resp's header and status code to w.
func copyResponseHeader(w http.ResponseWriter, resp *http.Response) {
	log.Println("start copyResponseHeader")
	if resp == nil {
		log.Println("null resp in copyResponseHeader")
		return
	}
	newHeader := w.Header()
	for key, values := range resp.Header {
		for _, v := range values {
			newHeader.Add(key, v)
		}
	}

	w.WriteHeader(resp.StatusCode)
	log.Println("finished copyResponseHeader")
	
}

func resub(restring string, input string, sub string) string {
	exp, err := regexp.Compile(restring)
	if err != nil {
		log.Fatalln("regexp compile error:", restring)
	}
	return exp.ReplaceAllString(input, sub)
}	

func makeFilterName(input string) string {
	// get rid of optional ;foo
	input = resub(`;.*`, input, "")
	// only keep safe chars so can be safely used as command line arg
	// and pathname
	input = resub(`[^a-zA-Z01-9/\+}\x00-\x20-]`, input, "")
	return "proxyfilter-" + strings.ToLower(strings.Replace(input, "/", "_", -1))
}

func headerNameToEnvKey(name string) string {
	name_exclude_class := `[()<>@,;:\\"/\[\]?={}\x00-\x20\x80-\xff]`
	exp, err := regexp.Compile(name_exclude_class)
	if err != nil {
		log.Fatalln("regexp compile error")
	}
	if exp.MatchString(name) {
		return ""
	}
	return "HTTP_" + strings.Replace(strings.ToUpper(name), "-", "_", -1)
}

func headersToEnv(h http.Header) []string {
	count := 0
	for _, values := range h {
		for _, _ = range values {
			count++
		}
	}
	out := make([]string, count)
	count = 0
	var hs string
	for key, values := range h {
		for _, v := range values {
			hs = headerNameToEnvKey(key)
			if hs != "" {
				out[count] =  hs + "=" + v
				count++
			}
		}
	}
	return out
}

func acceptGzip(req *icap.Request) bool {
	s := req.Header.Get("Accept-Encoding")
	return resub(`\bgzip\b`, s, "") != s
}

func handleIcap(w icap.ResponseWriter, req *icap.Request) {
	startTime := time.Now()
	h := w.Header()
	h.Set("ISTag", ISTag) // If this changes, the ICAP client (eg. Squid) will invalidate its cached responses from us.
	h.Set("Service", "Programmable proxy")

	switch req.Method {
	case "OPTIONS":
		h.Set("Methods", "RESPMOD")
		h.Set("Allow", "204")
		log.Println("got OPTIONS")
		w.WriteHeader(200, nil, false)
	case "RESPMOD":
		// pipe request through a program (in progress)
		rw := icap.NewBridgedResponseWriter(w)
		if req.Response == nil {
			log.Println("req.Response missing, skip")
		} else if filterName := makeFilterName(req.Response.Header.Get("Content-Type")); CommandInPath(filterName) == false {
			// no filter prog found, just copy everything
			log.Println("filter not found so passing through:", filterName)
			copyResponseHeader(rw, req.Response)
			io.Copy(rw, req.Response.Body) /* TBD check errors */
		} else {
			tenc := req.Response.Header.Get("Content-Transfer-Encoding")
			var do_gzip bool
			req.Response.Header.Del("Content-Length")
			if tenc == "" && acceptGzip(req) {
				req.Response.Header.Set("Content-Transfer-Encoding", "gzip")
				do_gzip = true
			} else {
				do_gzip = false
			}
			copyResponseHeader(rw, req.Response)
			var body io.Reader = req.Response.Body
			
			optstring := ""
			if do_gzip {
				optstring = "gzip"
			}
			
			PipeThrough(rw, body, headersToEnv(req.Response.Header), filterName, optstring)
			// can't send error response since headers already copied
		}
		elapsed := time.Since(startTime)
		log.Println("req processing time in msecs:", elapsed.Seconds() * 1000.0)
	default:
		w.WriteHeader(405, nil, false)
		log.Println("Invalid request method")
		
	}
}

// lifted from redwood
func logAccess(req *http.Request, resp *http.Response) {
	status := 0
	if resp != nil {
		status = resp.StatusCode
	}
	log.Println(time.Now().Format("2006-01-02 15:04:05"))
	log.Println(req.URL)
	log.Println(status)
	log.Println("\n")
}

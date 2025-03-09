#ifndef UTIL_H_
#define UTIL_H_

// Uncomment the following line to enable debug prints
#define AUTOCAM_DEBUG

#ifdef AUTOCAM_DEBUG
  #define LOG(...) Serial.print(__VA_ARGS__)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
  #define LOGLN(...) Serial.println(__VA_ARGS__)
#else
  #define LOG(...) ((void)0)
  #define LOGF(...) ((void)0)
  #define LOGLN(...) ((void)0)
#endif

#endif // UTIL_H_

#include "utils.h"

#include <errno.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <syscall.h>
#include <unistd.h>

long sched_setattr(pid_t pid, const struct sched_attr * attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

bool setProcessHighPiority()
{
  /* Get high priority */
  struct sched_attr attr;
  memset(&attr, 0, sizeof(attr));
  attr.size = sizeof(attr);
  attr.sched_policy = SCHED_RR;
  attr.sched_priority = 99;
  if(sched_setattr(0, &attr, 0) < 0)
  {
    printf("sched_setattr failed: %m\n");
    return false;
  }
  return true;
}

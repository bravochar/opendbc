#!/usr/bin/env bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR

source ../../../../setup.sh

GREEN="\e[1;32m"
YELLOW="\e[1;33m"
RED="\e[1;31m"
NC='\033[0m'

: "${CPPCHECK_DIR:=$DIR/cppcheck/}"

# install cppcheck if missing
if [ -z "${SKIP_CPPCHECK_INSTALL}" ]; then
  $DIR/install.sh
fi

# ensure checked in coverage table is up to date
if [ -z "$SKIP_TABLES_DIFF" ]; then
  python3 $CPPCHECK_DIR/addons/misra.py -generate-table > coverage_table
  if ! git diff --quiet coverage_table; then
    echo -e "${YELLOW}MISRA coverage table doesn't match. Update and commit:${NC}"
    exit 3
  fi
fi

cd $BASEDIR
if [ -z "${SKIP_BUILD}" ]; then
  scons -j8
fi

CHECKLIST=$DIR/checkers.txt
echo "Cppcheck checkers list from test_misra.sh:" > $CHECKLIST

cppcheck() {
  # get all gcc defines: arm-none-eabi-gcc -dM -E - < /dev/null
  COMMON_DEFINES="-D__GNUC__=9"

  # note that cppcheck build cache results in inconsistent results as of v2.13.0
  OUTPUT=$DIR/.output.log

  echo -e "\n\n\n\n\nTEST variant options:" >> $CHECKLIST
  echo -e ""${@//$BASEDIR/}"\n\n" >> $CHECKLIST # (absolute path removed)

  $CPPCHECK_DIR/cppcheck --inline-suppr -I $BASEDIR \
          -I "$(arm-none-eabi-gcc -print-file-name=include)" \
          --suppressions-list=$DIR/suppressions.txt --suppress=*:*inc/* \
          --suppress=*:*include/* --error-exitcode=2 --check-level=exhaustive --safety \
          --platform=arm32-wchar_t4 $COMMON_DEFINES --checkers-report=$CHECKLIST.tmp \
          --std=c11 "$@" 2>&1 | tee $OUTPUT

  cat $CHECKLIST.tmp >> $CHECKLIST
  rm $CHECKLIST.tmp
  # cppcheck bug: some MISRA errors won't result in the error exit code,
  # so check the output (https://trac.cppcheck.net/ticket/12440#no1)
  if grep -e "misra violation" -e "error" -e "style: " $OUTPUT > /dev/null; then
    printf "${RED}** FAILED: MISRA violations found!${NC}\n"
    exit 1
  fi
}

PANDA_OPTS=" --enable=all --enable=unusedFunction --addon=misra"

printf "\n${GREEN}** Safety **${NC}\n"
cppcheck $PANDA_OPTS -UCANFD $BASEDIR/opendbc/safety/main.c

printf "\n${GREEN}** Safety with CANFD **${NC}\n"
cppcheck $PANDA_OPTS -DCANFD $BASEDIR/opendbc/safety/main.c

printf "\n${GREEN}Success!${NC} took $SECONDS seconds\n"

# ensure list of checkers is up to date
cd $DIR
if [ -z "$SKIP_TABLES_DIFF" ] && ! git diff --quiet $CHECKLIST; then
  echo -e "\n${YELLOW}WARNING: Cppcheck checkers.txt report has changed. Review and commit...${NC}"
  exit 4
fi

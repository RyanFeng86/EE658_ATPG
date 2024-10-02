//Header file

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>

#define NUMFUNCS 6

#define MAXLINE 81               /* Input buffer size */
#define MAXNAME 31               /* File name size */

#define MAX 20



namespace ns1
{
	enum e_com {READ, PC, HELP, QUIT, LEV, LOGICSIM};
	enum e_state {EXEC, CKTLD};         /* Gstate values */
	enum e_ntype {GATE, PI, FB, PO};    /* column 1 of circuit format */
	enum e_gtype {IPT, BRCH, XOR, OR, NOR, NOT, NAND, AND};  /* gate types */  

	
	typedef struct n_struc {
		unsigned indx;             /* node index(from 0 to NumOfLine - 1 */
		unsigned num;              /* line number(May be different from indx */
		enum e_gtype type;         /* gate type */
		unsigned fin;              /* number of fanins */
		unsigned fout;             /* number of fanouts */
		struct n_struc **unodes;   /* pointer to array of up nodes */
		struct n_struc **dnodes;   /* pointer to array of down nodes */
		int level;                 /* level of the gate output */
		int val; 			  	   /* value of node */
	} NSTRUC;    

/*----------------- Command definitions ----------------------------------*/

	struct cmdstruc {
		char name[MAXNAME];        /* command syntax */
		int (*fptr)(char*);             /* function pointer of the commands */
		enum e_state state;        /* execution state sequence */
	};
	int cread(char*), pc(char*), help(char*), quit(char*), lev(char*), logicsim(char*);
	struct cmdstruc command[NUMFUNCS] = {
		{"READ", cread, EXEC},
		{"PC", pc, CKTLD},
		{"HELP", help, EXEC},
		{"QUIT", quit, EXEC},
		{"LEV", lev, CKTLD},
		{"LOGICSIM", logicsim, CKTLD}
	}; 
	
	char circuit_name[MAXLINE];

	
/*------------------------------------------------------------------------*/
	enum e_state Gstate = EXEC;     /* global exectution sequence */
	NSTRUC *Node;                   /* dynamic array of nodes */
	NSTRUC **Pinput;                /* pointer to array of primary inputs */
	NSTRUC **Poutput;               /* pointer to array of primary outputs */
	int Nnodes;                     /* number of nodes */
	int Npi;                        /* number of primary inputs */
	int Npo;                        /* number of primary outputs */
	int Done = 0;                   /* status bit to terminate program */
/*------------------------------------------------------------------------*/



	class sim_data //node
	{
		public:
			int time;
			struct n_struc **enodes;   /* pointer to array of events nodes */
			struct n_struc **anodes;   /* pointer to array of activated nodes */
			int add_activation(int line[], int value[]);
			int add_event(int line[], int value[]);
			sim_data(int line[], int value[]);
		//some data sturcture to hold list of events at time t
		//array [line][value]
		
		//some data struture to hold list of activations at time t
		//some data structure to values of lines at time t
	};

	class sim_node
	{
		public:
			sim_data sd; //red
			sim_node *next;
			sim_node(int line[], int value[]);
	};
	
	sim_node *start_node = NULL; //declares our first sim_node (initial values from txt file)
}
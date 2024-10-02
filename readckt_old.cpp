/*=======================================================================
  A simple parser for "self" format

  The circuit format (called "self" format) is based on outputs of
  a ISCAS 85 format translator written by Dr. Sandeep Gupta.
  The format uses only integers to represent circuit information.
  The format is as follows:

1        2        3        4           5           6 ...
------   -------  -------  ---------   --------    --------
0 GATE   outline  0 IPT    #_of_fout   #_of_fin    inlines
                  1 BRCH
                  2 XOR(currently not implemented)
                  3 OR
                  4 NOR
                  5 NOT
                  6 NAND
                  7 AND

1 PI     outline  0        #_of_fout   0

2 FB     outline  1 BRCH   inline

3 PO     outline  2 - 7    0           #_of_fin    inlines




                                    Author: Chihang Chen
                                    Date: 9/16/94

=======================================================================*/

/*=======================================================================
  - Write your program as a subroutine under main().
    The following is an example to add another command 'lev' under main()

enum e_com {READ, PC, HELP, QUIT, LEV};
#define NUMFUNCS 5
int cread(), pc(), quit(), lev();
struct cmdstruc command[NUMFUNCS] = {
   {"READ", cread, EXEC},
   {"PC", pc, CKTLD},
   {"HELP", help, EXEC},
   {"QUIT", quit, EXEC},
   {"LEV", lev, CKTLD},
};

lev()
{
   ...
}
=======================================================================*/
#include <limits.h>

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <cmath>
#include <string>     // std::string, std::stoi
#include <queue>
#include <bits/stdc++.h> 
#include <sys/time.h> 

using namespace std;

#define MAXLINE 81 /* Input buffer size */
#define MAXNAME 31 /* File name size */

#define MAXFAULT 1000

#define DEBUG_LUKE 0 /*FOR UART OUTPUTS*/

#define Upcase(x) ((isalpha(x) && islower(x)) ? toupper(x) : (x))
#define Lowcase(x) ((isalpha(x) && isupper(x)) ? tolower(x) : (x))

enum e_com
{
    READ,
    PC,
    HELP,
    QUIT,
    LEV,
    LOGICSIM,
    RFL,
    DFS,
    RTG,
    PFS,
    DALG,
    PODEM,
    ATPG,
    ATPG_DET,
};
enum e_state
{
    EXEC,
    CKTLD
}; /* Gstate values */
enum e_ntype
{
    GATE,
    PI,
    FB,
    PO
}; /* column 1 of circuit format */
enum e_gtype
{
    IPT,
    BRCH,
    XOR,
    OR,
    NOR,
    NOT,
    NAND,
    AND,
    XNOR
}; /* gate types */

enum e_val
{
	ZERO 	= 	0,
	ONE 	= 	1,
	X 		= 	2,
	D 		= 	3,
	D_bar	= 	4
};

struct cmdstruc
{
    char name[MAXNAME];  /* command syntax */
    int (*fptr)(char *); /* function pointer of the commands */
    enum e_state state;  /* execution state sequence */
};

int maxlevel;
string circuit_name;
	
/*----------------- Command definitions ----------------------------------*/
#define NUMFUNCS 14

struct n_struc;
typedef n_struc NSTRUC;
/*------------------------------------------------------------------------*/
class DFS_Node //node and SS@F associated with it to be used with DFS
{
public:
    NSTRUC *Dnode; //pointer to node
    int SSaF;      //0: boolean value for SS@0 fault. 1: boolean value for SS@1 fault
};
bool output_results = true;
/*
class F_List //fault list for each node
{
	public:
		NSTRUC *Lnode; //pointer to node of which the list is for
		DFS_Node DFS_List[MAXLINE]; //fault list made of up DFS Nodes
};

F_List Total_DFS_List[MAXLINE]; //Fault list for all nodes
*/

typedef struct n_struc//This part is important do not delete without Luck and Ryan agree
{
    unsigned indx;           /* node index(from 0 to NumOfLine - 1 */
    unsigned num;            /* line number(May be different from indx */
    enum e_gtype type;       /* gate type */
    unsigned fin;            /* number of fanins */
    unsigned fout;           /* number of fanouts */
    struct n_struc **unodes; /* pointer to array of up nodes */
    struct n_struc **dnodes; /* pointer to array of down nodes */
    int level;               /* level of the gate output */
    int val;                 /* value of node for simulation */
    //bool SSa0_det;			 /* SS@0 is detectable. For use with DFS */
    //bool SSa1_det;			 /* SS@1 is detectable. For use with DFS */
    DFS_Node DFS_List[MAXFAULT]; /* Node Fault List */
    
    bool initial_sign;
	enum e_val five_val;       /* five value line value used for ATPG */


    //Ryan's part do not modify without notice
    unsigned int valp;
    bool Vval;                   
    bool Uval;
    bool Vval_fault;
    bool Uval_fault;
    unsigned fin_lev;



} NSTRUC;

/*------------Fault Results------------------*/
vector<string> Faults_Covered;
vector<string> RTG_Faults_Covered;

/*----------------- RFL Structure ----------------------------------*/
class RFL_Node
{
public:
    NSTRUC *Cnode; //pointer to checkpoint node
    bool SSa0;     //boolean value for SS@0 fault
    bool SSa1;     //boolean value for SS@1 fault
};

//RFL_Node RFL_List[MAXLINE];
int fault_count;
/*------------------------------------------------------------------------*/

enum e_state Gstate = EXEC; /* global exectution sequence */
NSTRUC *Node;               /* dynamic array of nodes */
NSTRUC **Pinput;            /* pointer to array of primary inputs */
NSTRUC **Poutput;           /* pointer to array of primary outputs */
int Nnodes;                 /* number of nodes */
int Npi;                    /* number of primary inputs */
int Npo;                    /* number of primary outputs */
int Done = 0;               /* status bit to terminate program */
                            /*------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: shell
description:
  This is the main program of the simulator. It displays the prompt, reads
  and parses the user command, and calls the corresponding routines.
  Commands not reconized by the parser are passed along to the shell.
  The command is executed according to some pre-determined sequence.
  For example, we have to read in the circuit description file before any
  action commaif(command[com].state <= Gstate) (*command[com].fptr)(cp);nds.  The code uses "Gstate" to check the execution
  sequence.
  Pointers to functions are used to make function calls which makes the
  code short and clean.
-----------------------------------------------------------------------*/
char line[100], title[100];

#define Undefine INT_MIN

//Levelization
vector< vector<NSTRUC*> > lev_array;
int lev(char *cp)
{
    int Ngate = 0;
    int g = 0;
    char m[100] = {};
    int i;
    for (i = 1; i < 100; i++)
    {
        if (line[i] == '\n')
            break;
        else
        {
            m[g] = line[i];
            g++;
        }
    }

    FILE *fpp;
    fpp = fopen(m, "w");

    fprintf(fpp, title);
    fprintf(fpp, "\n");
    fprintf(fpp, "#PI: %d\n", Npi);
    fprintf(fpp, "#PO: %d\n", Npo);
    fprintf(fpp, "#Nodes: %d\n", Nnodes);
    //printf("#Gates: %d\n",Undefine);
    NSTRUC *np;

    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        np->level = Undefine;
        if (np->type != IPT && np->type != BRCH)
            Ngate++;
    }
    fprintf(fpp, "#Gates: %d\n", Ngate);

    int k = 1;
    while (k)
    {
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (((np->level) < 0) && (np->type == IPT))
                np->level = 0;
            else
            {
                if (np->level < 0)
                {
                    int in = 0, plev = 0;
                    int j;
                    for (j = 0; j < np->fin; j++)
                    {
                        if (np->unodes[j]->level >= 0)
                        {
                            in++;
                            if (np->unodes[j]->level > plev)
                                plev = np->unodes[j]->level;
                        }
                    }
                    if (in == np->fin && (np->type == BRCH || np->type == XOR || np->type == OR || np->type == NOR || np->type == NOT || np->type == NAND || np->type == AND || np->type == XNOR))
                    {
                        np->level = plev + 1;
                        if (np->level > maxlevel)
                            maxlevel = np->level;
                    }
                }
            }
        }

        k = 0;
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (np->level < 0)
                k++;
        }
    }

    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];

        fprintf(fpp, "%d %d\n", np->num, Node[i].level);
    }
    fclose(fpp);
    return 0;
}

//logic_simulation
int logicsim(char *cp)
{

    int g = 0;
    int i;
    char infilename[100] = {};
    char outfilename[100] = {};
    NSTRUC *np;
    int maxlev;

    //Get the maximum level of the circuit
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        np->level = Undefine; //initial each nodes' level
        np->val = Undefine;   //initial each nodes' val
    }

    int k = 1;
    while (k)
    {
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (((np->level) < 0) && (np->type == IPT))
                np->level = 0;
            else
            {
                if (np->level < 0)
                {
                    int in = 0, plev = 0;
                    int j;
                    for (j = 0; j < np->fin; j++)
                    {
                        if (np->unodes[j]->level >= 0)
                        {
                            in++;
                            if (np->unodes[j]->level > plev)
                                plev = np->unodes[j]->level;
                        }
                    }
                    if (in == np->fin && (np->type == BRCH || np->type == XOR || np->type == OR || np->type == NOR || np->type == NOT || np->type == NAND || np->type == AND || np->type == XOR || np->type == XNOR))
                        np->level = plev + 1;
                    if (np->level > maxlev)
                        maxlev = np->level;
                }
            }
        }

        k = 0;
        for (i = 0; i < Nnodes; i++) //To determine whether to exit the while loop
        {
            np = &Node[i];
            if (np->level < 0)
                k++;
        }
    }
    //maximum-circuit-level get ends

    //process file name
    for (i = 1; i < 100; i++)
    {
        if (line[i] == ' ')
            break;
        else
        {
            infilename[g] = line[i];
            g++;
        }
    }
    i++;
    g = 0;
    for (i; i < 100; i++)
    {
        if (line[i] == '\n')
            break;
        else
        {
            outfilename[g] = line[i];
            g++;
        }
    }
    //file name process end

    /*
    FILE *fd;
    int nd, vl;
    if ((fd = fopen(infilename, "r")) == NULL)
    {
        printf("File %s does not exist!\n", infilename);
        return 0;
    }
    else
    {
        while (fgets(infilename, MAXLINE, fd) != NULL)
        {
            if (sscanf(infilename, "%d,%d", &nd, &vl) == 2)
            {
                int i = 0;

                for (i; i < Nnodes; i++)
                {
                    np = &Node[i];
                    if (np->num == nd)
                    {
                        np->val = vl;
                        break;
                    }
                }

                //printf("%d %d\n",nd,vl);
            }
        }
    }
    fclose(fd);
    */

    //read in test file
    vector<int> Pin;
    vector< vector<int> > Pvalue;
    ifstream infile(infilename, ios::in);

    if (!infile)
        cout << "File " << infilename << " does not exist!" << endl;
    else
    {
        char tmp[1024];
        infile.getline(tmp, sizeof(tmp));
        int i;
        string pp = "";
        for (i = 0; i < sizeof(tmp); i++)
        {
            if (tmp[i] == '\0')
            {
                Pin.push_back(atoi(pp.c_str()));
                pp = "";
                break;
            }
            else if (tmp[i] != ',')
                pp.push_back(tmp[i]); //initial input pin number
            else
            {
                Pin.push_back(atoi(pp.c_str()));
                pp = "";
            }
        }
        int j = 0;
        while (infile.getline(tmp, sizeof(tmp)))
        {
            Pvalue.push_back({});
            for (i = 0; i < sizeof(tmp); i++)
            {
                if (tmp[i] == '\0')
                    break;
                else
                {
                    if (tmp[i] != ',')
                        Pvalue[j].push_back((tmp[i] - '0')); //initial all coherence input pin value
                }
            }
            j++;
        }
    }
    infile.close();
    //read in test file

    //
    //logicsim part
    //
    //propagate value
    int pp_tms;                                 //define propagate times,based on test vector number
    vector<int> outputp;                        //creat an vector to save output pin number
    vector< vector<int> > outputv(Pvalue.size()); //creat an vector to save output value
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->dnodes[0] == NULL)
        {
            outputp.push_back(np->num);
            cout << np->num << endl;
        }
    }

    cout << "********" << Pvalue.size() << "********" << endl;
    for (pp_tms = 0; pp_tms < Pvalue.size(); pp_tms++)
    {
        //initial each node value
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            np->val = Undefine; //initial each nodes' val
        }

        int cc = 0;
        for (i = 0; i < Pin.size(); i++)
        {
            int j;
            for (j = 0; j < Nnodes; j++)
            {
                np = &Node[j];
                if (np->num == Pin[i])
                {
                    np->val = Pvalue[pp_tms][cc]; //put test vector's each line value to corespondent node

                    cc++;
                    break;
                }
            }
        }

        int level = 1;
        int j;
        int detect;
        //printf("max level is: %0d", maxlev);
        for (level; level <= maxlev; level++)
        {
            for (i = 0; i < Nnodes; i++)
            {
                np = &Node[i];

                for (j = 0; j < np->fin; j++)
                {
                    detect = 1;
                    if (np->unodes[j]->val < 0)
                    {
                        detect = 0;
                    }
                }

                np = &Node[i];
                if (np->level == level && detect)
                {
                    int c_count = 0;
                    int count_one = 0;
                    int count_x = 0;
                    switch (np->type)
                    {
                    case BRCH: //done
                        np->val = np->unodes[0]->val;
                        break;

                    case XNOR: //done
                        for (c_count = 0; c_count < np->fin; c_count++)
                        {
                            if (np->unodes[c_count]->val == 0)
                                count_one++;
                            if (np->unodes[c_count]->val == 40)
                                count_x++;
                        }
                        count_one = count_one % 2;
                        if (count_x)
                            np->val = 40;
                        else if (count_one == 1)
                            np->val = 0;
                        else
                            np->val = 1;
                        break;

                    case XOR: //done

                        for (c_count = 0; c_count < np->fin; c_count++)
                        {
                            if (np->unodes[c_count]->val == 1)
                                count_one++;
                            if (np->unodes[c_count]->val == 40)
                                count_x++;
                        }
                        count_one = count_one % 2;
                        if (count_x)
                            np->val = 40;
                        else if (count_one == 1)
                            np->val = 1;
                        else
                            np->val = 0;
                        break;

                    case OR: //done
                        for (c_count = 0; c_count < np->fin; c_count++)
                        {
                            if (np->unodes[c_count]->val == 1)
                                count_one++;
                            if (np->unodes[c_count]->val == 40)
                                count_x++;
                        }
                        if (count_one)
                            np->val = 1;
                        else if (count_x)
                            np->val = 40;
                        else
                            np->val = 0;
                        break;

                    case NOR: //done
                        for (c_count = 0; c_count < np->fin; c_count++)
                        {
                            if (np->unodes[c_count]->val == 1)
                                count_one++;
                            if (np->unodes[c_count]->val == 40)
                                count_x++;
                        }
                        if (count_one)
                            np->val = 0;
                        else if (count_x)
                            np->val = 40;
                        else
                            np->val = 1;
                        break;

                    case NOT: //done
                        if (np->unodes[0]->val == 0)
                            np->val = 1;
                        else if (np->unodes[0]->val == 40)
                            np->val = 40;
                        else
                            np->val = 0;
                        break;

                    case NAND: //done
                        for (c_count = 0; c_count < np->fin; c_count++)
                        {
                            if (np->unodes[c_count]->val == 0)
                                count_one++;
                            if (np->unodes[c_count]->val == 40)
                                count_x++;
                        }
                        if (count_one)
                            np->val = 1;
                        else if (count_x)
                            np->val = 40;
                        else
                            np->val = 0;
                        break;

                    case AND: //done
                        for (c_count = 0; c_count < np->fin; c_count++)
                        {
                            if (np->unodes[c_count]->val == 0)
                                count_one++;
                            if (np->unodes[c_count]->val == 40)
                                count_x++;
                        }
                        if (count_one)
                            np->val = 0;
                        else if (count_x)
                            np->val = 40;
                        else
                            np->val = 1;
                        break;

                    default:
                        printf("No found gate type\n");
                    }
                }
            }
        }

        for (i = 0; i < outputp.size(); i++)
        {
            for (j = 0; j < Nnodes; j++)
            {
                np = &Node[j];
                if (np->num == outputp[i])
                {
                    outputv[pp_tms].push_back(np->val);
                    break;
                }
            }
        }
		std::cout << "Max level is " << maxlev<< std::endl;
    }

    //generate output into file
    /*
    fd = fopen(outfilename, "w");
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->dnodes[0] == NULL)
        {
            fprintf(fd, "%d,%d\n", np->num, Node[i].val);
        }
    }
    fclose(fd);
    for (int j = 0; j < Nnodes; j++)
    {
        np = &Node[j];
        printf("node %d has value %d\n", np->num, np->val);
    }*/

    //write the results to file
    ofstream outfile(outfilename);
    for (i = 0; i < outputp.size(); i++)
    {
        outfile << outputp[i];
        if ((i + 1) < outputp.size())
            outfile << ",";
    }
    outfile << endl;
    for (i = 0; i < outputv.size(); i++)
    {
        int j;
        for (j = 0; j < outputv[i].size(); j++)
        {
            if (outputv[i][j] == 40)
                outfile << "X";
            else
                outfile << outputv[i][j];
            if ((j + 1) < outputv[i].size())
                outfile << ",";
        }
        if ((i + 1) < outputv.size())
            outfile << endl;
    }
    outfile.close();
    //write the results to file end

    //release vector
    vector<int>().swap(Pin);
    vector<vector<int> >().swap(Pvalue);
    vector<int>().swap(outputp);
    vector<vector<int> >().swap(outputv);
    
}
/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: cread
description:
  This routine clears the memory space occupied by the previous circuit
  before reading in new one. It frees up the dynamic arrays Node.unodes,
  Node.dnodes, Node.flist, Node, Pinput, Poutput, and Tap.
-----------------------------------------------------------------------*/
void clear()
{
    int i;

    for (i = 0; i < Nnodes; i++)
    {
        free(Node[i].unodes);
        free(Node[i].dnodes);
    }
    free(Node);
    free(Pinput);
    free(Poutput);
    Gstate = EXEC;
    Faults_Covered.clear();
    RTG_Faults_Covered.clear();
	output_results = true;
	
}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: cread
description:
  This routine allocatess the memory space required by the circuit
  description data structure. It allocates the dynamic arrays Node,
  Node.flist, Node, Pinput, Poutput, and Tap. It also set the default
  tap selection and the fanin and fanout to 0.
-----------------------------------------------------------------------*/
void allocate()
{
    int i;

    Node = (NSTRUC *)malloc(Nnodes * sizeof(NSTRUC));
    Pinput = (NSTRUC **)malloc(Npi * sizeof(NSTRUC *));
    Poutput = (NSTRUC **)malloc(Npo * sizeof(NSTRUC *));
    for (i = 0; i < Nnodes; i++)
    {
        Node[i].indx = i;
        Node[i].fin = Node[i].fout = 0;
    }
}

/*-----------------------------------------------------------------------
input: circuit description file name
output: nothing
called by: main
description:
  This routine reads in the circuit description file and set up all the
  required data structure. It first checks if the file exists, then it
  sets up a mapping table, determines the number of nodes, PI's and PO's,
  allocates dynamic data arrays, and fills in the structural information
  of the circuit. In the ISCAS circuit description format, only upstream
  nodes are specified. Downstream nodes are implied. However, to facilitate
  forward implication, they are also built up in the data structure.
  To have the maximal flexibility, three passes through the circuit file
  are required: the first pass to determine the size of the mapping table
  , the second to fill in the mapping table, and the third to actually
  set up the circuit information. These procedures may be simplified in
  the future.
-----------------------------------------------------------------------*/
int cread(char *cp)
{
    char buf[MAXLINE];
    int ntbl, *tbl, i, j, k, nd, tp, fo, fi, ni = 0, no = 0;
    FILE *fd;
    NSTRUC *np;

    sscanf(cp, "%s", buf);
    int jj;
    int g = 0;
    int sig = 0;
    for (jj = 0; jj < 100; jj++)
        title[jj] = '\0';
    for (jj = 0; jj < 100; jj++)
    {
        if (cp[jj] == ' ')
            sig = 1;
        else
        {
            if (sig == 1 && cp[jj] != '.')
            {
                title[g] = cp[jj];
                g++;
            }
            else if (sig == 0)
                continue;
            else
                break;
        }
    }
	circuit_name = "";
	int ndx = 0;
	while (buf[ndx] != '.')
	{
		circuit_name += buf[ndx];
		ndx++;
	}
	std::cout << "Circuit is: " << circuit_name << std::endl;

    if ((fd = fopen(buf, "r")) == NULL)
    {
        printf("File %s does not exist!\n", buf);
        return 0;
    }
    if (Gstate >= CKTLD)
        clear();
    Nnodes = Npi = Npo = ntbl = 0;
    while (fgets(buf, MAXLINE, fd) != NULL)
    {
        if (sscanf(buf, "%d %d", &tp, &nd) == 2)
        {
            if (ntbl < nd)
                ntbl = nd;
            Nnodes++;
            if (tp == PI)
                Npi++;
            else if (tp == PO)
                Npo++;
        }
    }
    tbl = (int *)malloc(++ntbl * sizeof(int));

    fseek(fd, 0L, 0);
    i = 0;
    while (fgets(buf, MAXLINE, fd) != NULL)
    {
        if (sscanf(buf, "%d %d", &tp, &nd) == 2)
            tbl[nd] = i++;
    }
    allocate();

    fseek(fd, 0L, 0);
    while (fscanf(fd, "%d %d", &tp, &nd) != EOF)
    {
        np = &Node[tbl[nd]];
        np->num = nd;
        if (tp == PI)
            Pinput[ni++] = np;
        else if (tp == PO)
            Poutput[no++] = np;
        switch (tp)
        {
        case PI:
        case PO:
        case GATE:
            fscanf(fd, "%d %d %d", &np->type, &np->fout, &np->fin);
            break;

        case FB:
            np->fout = np->fin = 1;
            fscanf(fd, "%d", &np->type);
            break;

        default:
            printf("Unknown node type!\n");
            exit(-1);
        }
        np->unodes = (NSTRUC **)malloc(np->fin * sizeof(NSTRUC *));
        np->dnodes = (NSTRUC **)malloc(np->fout * sizeof(NSTRUC *));
        for (i = 0; i < np->fin; i++)
        {
            fscanf(fd, "%d", &nd);
            np->unodes[i] = &Node[tbl[nd]];
        }
        for (i = 0; i < np->fout; np->dnodes[i++] = NULL)
            ;
    }
    for (i = 0; i < Nnodes; i++)
    {
        for (j = 0; j < Node[i].fin; j++)
        {
            np = Node[i].unodes[j];
            k = 0;
            while (np->dnodes[k] != NULL)
                k++;
            np->dnodes[k] = &Node[i];
        }
    }
    fclose(fd);
    Gstate = CKTLD;
    printf("==> OK\n");
}

/*-----------------------------------------------------------------------
input: gate type
output: string of the gate type
called by: pc
description:
  The routine receive an integer gate type and return the gate type in
  character string.
-----------------------------------------------------------------------*/
char *gname(int tp)
{
    switch (tp)
    {
    case 0:
        return ("PI");
    case 1:
        return ("BRANCH");
    case 2:
        return ("XOR");
    case 3:
        return ("OR");
    case 4:
        return ("NOR");
    case 5:
        return ("NOT");
    case 6:
        return ("NAND");
    case 7:
        return ("AND");
    case 8:
        return ("XNOR");
    }
}

/*-----------------------------------------------------------------------
input: five value
output: string of the gate type
called by: dalg
description:
  The routine receive an integer gate type and return the five value in
  character string.
-----------------------------------------------------------------------*/
char *fval(int tp)
{
    switch (tp)
    {
    case 0:
        return ("0");
    case 1:
        return ("1");
    case 2:
        return ("X");
    case 3:
        return ("D");
    case 4:
        return ("D bar");
    }
}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: main
description:
  The routine prints out the circuit description from previous READ command.
-----------------------------------------------------------------------*/
int pc(char *cp)
{
    int i, j;
    NSTRUC *np;
    //char *gname();

    printf(" Node   Type \tIn     \t\t\tOut    \n");
    printf("------ ------\t-------\t\t\t-------\n");
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        printf("\t\t\t\t\t");
        for (j = 0; j < np->fout; j++)
            printf("%d ", np->dnodes[j]->num);
        printf("\r%5d  %s\t", np->num, gname(np->type));
        for (j = 0; j < np->fin; j++)
            printf("%d ", np->unodes[j]->num);
        printf("\n");
    }
    printf("Primary inputs:  ");
    for (i = 0; i < Npi; i++)
        printf("%d ", Pinput[i]->num);
    printf("\n");
    printf("Primary outputs: ");
    for (i = 0; i < Npo; i++)
        printf("%d ", Poutput[i]->num);
    printf("\n\n");
    printf("Number of nodes = %d\n", Nnodes);
    printf("Number of primary inputs = %d\n", Npi);
    printf("Number of primary outputs = %d\n", Npo);
}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: main 
description:
  The routine prints ot help inormation for each command.
-----------------------------------------------------------------------*/
int help(char *cp)
{
    printf("READ filename - ");
    printf("read in circuit file and creat all data structures\n");
    printf("PC - ");
    printf("print circuit information\n");
    printf("HELP - ");
    printf("print this help information\n");
    printf("QUIT - ");
    printf("stop and exit\n");
}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: main 
description:
  Set Done to 1 which will terminates the program.
-----------------------------------------------------------------------*/
int quit(char *cp)
{
    Done = 1;
}

/*======================================================================*/

//--------------------------------------------------------------------------------------
int rfl(char *cp)
{
    char buf[MAXLINE];
    sscanf(cp, "%s", buf);
    NSTRUC *np;
    FILE *fd;
    fd = fopen(buf, "w");
    fault_count = 0;
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->type == 0 || np->type == 1)
        {
            fault_count++;
            if (DEBUG_LUKE) printf("Node %d is a %s. Number of checkpoint is %0d\n", np->num, gname(np->type), fault_count);
        }
    }
    RFL_Node RFL_List[fault_count];
    int count = 0;
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->type == 0 || np->type == 1)
        {
            if (DEBUG_LUKE) printf("Node %d is a %s. Adding to RFL\n", np->num, gname(np->type));
            RFL_List[count].Cnode = np;
            RFL_List[count].SSa0 = true;
            RFL_List[count].SSa1 = true;
            if (DEBUG_LUKE) printf("Node %0d has SS@0: %s and SS@1: %s.\n", RFL_List[count].Cnode->num,
                   RFL_List[count].SSa0 ? "true" : "false",
                   RFL_List[count].SSa1 ? "true" : "false");
            count++;
        }
    }
    for (int j = 0; j < fault_count; j++)
    {
        if (RFL_List[j].SSa0)
            fprintf(fd, "%d@0\n", RFL_List[j].Cnode->num);
        if (RFL_List[j].SSa1)
            fprintf(fd, "%d@1\n", RFL_List[j].Cnode->num);
    }
    fclose(fd);
    printf("RFL function called on circuit");
}
//-----------------------------------------------------------------------------

int list_union(NSTRUC *np)
{
    int list_ndx = 1;

    int j = 0;
    int duplicate = 0;
    for (int i = 0; i < np->fin; i++)
    {
        j = 0;
        while (np->unodes[i]->DFS_List[j].SSaF != -1) //traverse list of us node
        {
            //check if fault is in list already
            for (int k = 0; k < list_ndx; k++)
            {
                if (np->DFS_List[k].Dnode == np->unodes[i]->DFS_List[j].Dnode)
                {
                    duplicate = 1;
                    if (DEBUG_LUKE) printf("Found duplicate element %dSS@%d\n", np->unodes[i]->DFS_List[j].Dnode->num, np->unodes[i]->DFS_List[j].SSaF);
                    if (DEBUG_LUKE) printf("Found duplicate element %dSS@%d\n", np->unodes[i]->DFS_List[j].Dnode->num, np->unodes[i]->DFS_List[j].SSaF);
                }
            }
            if (duplicate == 0)
            {
                np->DFS_List[list_ndx] = np->unodes[i]->DFS_List[j]; //add fault to list;
                if (DEBUG_LUKE) printf("Adding fault %dSS@%d to node %d\n", np->unodes[i]->DFS_List[j].Dnode->num, np->unodes[i]->DFS_List[j].SSaF, np->num);
                j++;
                list_ndx++;
            }
            else
            {
                j++;
            }
            duplicate = 0;
        }
    }
    if (DEBUG_LUKE) printf("Fault list for %d: \n", np->num);
    j = 0;
    while (np->DFS_List[j].SSaF != -1)
    {
        if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list of line %d\n", np->DFS_List[j].Dnode->num, np->DFS_List[j].SSaF, np->num);
        j++;
    }
}

int list_combination(NSTRUC *np, int c_val) //pass node which we want to calculate list for (in case of at least 1 NC value but not all)
{
    DFS_Node F_Intersection[MAXFAULT]; //hold intersection of all controlling values
    DFS_Node F_Union[MAXFAULT];        //hold intersection of all controlling values
    struct n_struc **nc_node;         //nodes that hold non-controlling values
    struct n_struc **c_node;          //nodes that hold controlling values
    int nc_count = 0;                 //num of nc values used for indexing nc_node
    int c_count = 0;                  //num of contorling values used for indexing c_node
    int list_ndx = 0;

    nc_node = (NSTRUC **)malloc(np->fin * sizeof(NSTRUC *)); //allocate memory for nodes
    c_node = (NSTRUC **)malloc(np->fin * sizeof(NSTRUC *));
    if (DEBUG_LUKE) printf("Allocated memory for nc_node and c_node\n");

    for (int i = 0; i < MAXFAULT; i++) //clear Dnode of intersection list
    {
        F_Intersection[i].Dnode = NULL;
        F_Union[i].Dnode = NULL;
    }

    for (int i = 0; i < np->fin; i++) //fill up nc/c node pointers
    {
        if (np->unodes[i]->val == c_val)
        {
            if (DEBUG_LUKE) printf("Node %d has controlling value %d. Adding to c_node.\n", np->unodes[i]->num, c_val);
            c_node[c_count] = np->unodes[i];
            c_count++;
        }
        else
        {
            if (DEBUG_LUKE) printf("Node %d has non-controlling value. Adding to nc_node.\n", np->unodes[i]->num);
            nc_node[nc_count] = np->unodes[i];
            nc_count++;
        }
    }
    if (DEBUG_LUKE) printf("Line %d has %d controlling values and %d non-controlling values\n", np->val, c_count, nc_count);

    if (DEBUG_LUKE) printf("Fault list of c_node[0] is: \n");

    while (c_node[0]->DFS_List[list_ndx].Dnode != NULL)
    {
        F_Intersection[list_ndx] = c_node[0]->DFS_List[list_ndx];
        if (DEBUG_LUKE) printf("Line %d has fault: %dSS@%d\n", c_node[0]->num, c_node[0]->DFS_List[list_ndx].Dnode->num, c_node[0]->DFS_List[list_ndx].SSaF);
        list_ndx++;
    }
    for (int i = 1; i < c_count; i++) //loop though controlling values
    {
        if (DEBUG_LUKE) printf("calculating interesction of all controlling value gates\n");
        int j = 0;
        while (F_Intersection[j].Dnode != NULL) //loop though faults in first list
        {
            int k = 0;
			bool dup = false;
            while (c_node[i]->DFS_List[k].Dnode != NULL)
            {

                if (DEBUG_LUKE) printf("Comparing %dSSa%d to %dSSa%d:  ", F_Intersection[j].Dnode->num, F_Intersection[j].SSaF, c_node[i]->DFS_List[k].Dnode->num, c_node[i]->DFS_List[k].SSaF);
                if (F_Intersection[j].Dnode == c_node[i]->DFS_List[k].Dnode)
                {
                    if (DEBUG_LUKE) printf("match! \n");
					dup = true;
                }
				
				/*
                else
                {
                    printf("no match! removing from intersection list\n");
                    F_Intersection[j].Dnode = NULL;
                }
				*/
                k++;
            }
			if (!dup)
			{
				if (DEBUG_LUKE) printf("no match! removing from intersection list\n");
                F_Intersection[j].Dnode = NULL;
			}
				
            j++;
			
        }
    }

    //find union of NC values
    int j = 0;
    int duplicate = 0;
    int u_ndx = 0;
    for (int i = 0; i < nc_count; i++) //for each nc node
    {
        j = 0;
        while (nc_node[i]->DFS_List[j].Dnode != NULL) //for each valud fault in the node
        {
            if (DEBUG_LUKE) printf("got here \n");
            for (int k = 0; k < u_ndx; k++) //for each fault in union list
            {
                if (F_Union[k].Dnode == nc_node[i]->DFS_List[j].Dnode)
                {
                    duplicate = 1;
                    if (DEBUG_LUKE) printf("Found duplicate element %dSS@%d", nc_node[i]->DFS_List[j].Dnode->num, nc_node[i]->DFS_List[j].SSaF);
                }
            }
            if (duplicate == 0)
            {
                F_Union[u_ndx] = nc_node[i]->DFS_List[j]; //add fault to list;
                if (DEBUG_LUKE) printf("Adding fault %dSS@%d to union list\n", nc_node[i]->DFS_List[j].Dnode->num, nc_node[i]->DFS_List[j].SSaF);
                j++;
                u_ndx++;
            }
            else
            {
                j++;
            }
            duplicate = 0;
        }
    }
    if (DEBUG_LUKE) printf("Union Fault list for non-controlling values:\n");
    j = 0;
    while (F_Union[j].Dnode != NULL)
    {
        if (DEBUG_LUKE) printf("Fault %dSS@%d is in the union fault list\n", F_Union[j].Dnode->num, F_Union[j].SSaF);
        j++;
    }
    j = 0;
    //Perform F_Intersection - F_Union
    for (int i = 0; i < MAXFAULT; i++)
    {
        j = 0;
        if (F_Intersection[i].Dnode != NULL)
        {
            while (F_Union[j].Dnode != NULL)
            {
                if (F_Intersection[i].Dnode == F_Union[j].Dnode)
                {
                    //remove from intersection list
                    F_Intersection[i].Dnode = NULL;
                }
                j++;
            }
        }
    }

    list_ndx = 1;
    for (int i = 0; i < MAXFAULT; i++)
    {
        if (F_Intersection[i].Dnode != NULL)
        {
            np->DFS_List[list_ndx] = F_Intersection[i];
            list_ndx++;
        }
    }
}

int list_subtraction(NSTRUC *np1, NSTRUC *np2, NSTRUC *np) //np1->DFS_List - np2->DFS_List += np->DFS_List
{
    DFS_Node F_Intersection[MAXFAULT]; //FL A - (FLA intersect FL B)
    int i = 0;
    int f_ndx = 0;
    int np1_size = 0;
    if (DEBUG_LUKE) printf("subtracting stuff\n");
    while (np1->DFS_List[np1_size].SSaF != -1) //find common elements in np1 and np2 fault lists
    {
        i = 0;
        while (np2->DFS_List[i].SSaF != -1 && np2->DFS_List[i].Dnode != NULL)
        {
            if (DEBUG_LUKE) printf("Comparing element %d of np2 with element %d of np1. Dnode of np1 is %d Dnode of np2 is %d\n", i, np1_size, np1->DFS_List[np1_size].Dnode, np2->DFS_List[i].Dnode);
            if (np1->DFS_List[np1_size].Dnode == np2->DFS_List[i].Dnode) //common element
            {
                if (DEBUG_LUKE) printf("np1 line %d and np2 line %d have common fault %dSS@%d", np1->num, np2->num, np1->DFS_List[np1_size].Dnode->num, np1->DFS_List[np1_size].SSaF);
                F_Intersection[f_ndx] = np1->DFS_List[np1_size];
                f_ndx++;
            }
            i++;
        }
        np1_size++;
    }
    NSTRUC *tempnp;
    int list_ndx = 1;
    int j = 0;
    //np list += np1 - f_intersection
    if (F_Intersection[0].SSaF = -1)
    {
        if (DEBUG_LUKE) printf("No intersection in inpuit lists.\n");
        while (np1->DFS_List[j].SSaF != -1) //traverse list of us node
        {
            np->DFS_List[list_ndx] = np1->DFS_List[j]; //add fault to list;
            if (DEBUG_LUKE) printf("Adding fault %dSS@%d to node %d\n", np1->DFS_List[j].Dnode->num, np1->DFS_List[j].SSaF, np->num);
            j++;
            list_ndx++;
        }
    }
    else
    {
        for (int k = 0; k < np1_size; k++)
        {
            if (DEBUG_LUKE) printf("There is SS@F intersection. Fix this\n");
        }
    }
    if (DEBUG_LUKE) printf("Fault list for %d: \n", np->num);
    j = 0;
    while (np->DFS_List[j].SSaF != -1)
    {
        if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list of line %d\n", np->DFS_List[j].Dnode->num, np->DFS_List[j].SSaF, np->num);
        j++;
    }
}
//------------------------------------------------------------------------------

//To be called by dfs. DO NOT MODIFY UNLESS LUKE SAYS SO
int logicsim_single(vector<int> p_in, vector<int> test_vect, char *cp)
{
    int Ngate = 0;
    int g = 0;
    int i;
    NSTRUC *np;

    lev(cp);

    //propagate value
    int level = 1;
    int j;
    int detect;
    if (DEBUG_LUKE) printf("max level is: %0d \n", maxlevel);
    for (level; level <= maxlevel; level++)
    {
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];

            for (j = 0; j < np->fin; j++)
            {
                detect = 1;
                if (np->unodes[j]->val < 0)
                {
                    detect = 0;
                }
            }

            np = &Node[i];
            if (np->level == level && detect)
            {
                int c_count = 0;
                int count_one = 0;
                if (DEBUG_LUKE) printf("evaulating line %d that has %d gate upstream\n", np->num, np->type);
                switch (np->type)
                {
                case BRCH:
                    np->val = np->unodes[0]->val;
                    break;
                case XNOR:

                    for (c_count = 0; c_count < np->fin; c_count++)
                    {
                        if (np->unodes[c_count]->val == 0)
                            count_one++;
                    }
                    count_one = count_one % 2;
                    if (count_one == 1)
                        np->val = 0;
                    else
                        np->val = 1;

                    break;
                case XOR:

                    for (c_count = 0; c_count < np->fin; c_count++)
                    {
                        if (np->unodes[c_count]->val == 1)
                            count_one++;
                    }
                    count_one = count_one % 2;
                    if (count_one == 1)
                        np->val = 1;
                    else
                        np->val = 0;
                    break;
                case OR:
                    for (c_count = 0; c_count < np->fin; c_count++)
                    {
                        if (np->unodes[c_count]->val == 1)
                            count_one++;
                    }
                    if (count_one)
                        np->val = 1;
                    else
                        np->val = 0;
                    break;
                case NOR:
                    for (c_count = 0; c_count < np->fin; c_count++)
                    {
                        if (np->unodes[c_count]->val == 1)
                            count_one++;
                    }
                    if (count_one)
                        np->val = 0;
                    else
                        np->val = 1;
                    break;
                case NOT:
                    if (np->unodes[0]->val == 0)
                        np->val = 1;
                    else
                        np->val = 0;
                    break;
                case NAND:
                    for (c_count = 0; c_count < np->fin; c_count++)
                    {
                        if (np->unodes[c_count]->val == 0)
                            count_one++;
                    }
                    if (count_one)
                        np->val = 1;
                    else
                        np->val = 0;
                    break;
                case AND:
                    for (c_count = 0; c_count < np->fin; c_count++)
                    {
                        if (np->unodes[c_count]->val == 0)
                            count_one++;
                    }
                    if (count_one)
                        np->val = 0;
                    else
                        np->val = 1;
                    break;
                default:
                    if (DEBUG_LUKE) printf("No found gate type\n");
                }
            }
        }
    }
    for (int j = 0; j < Nnodes; j++)
    {
        np = &Node[j];
        if (DEBUG_LUKE) printf("node %d has value %d\n", np->num, np->val);
    }
}

vector<string> dfs_single(vector<int> p_in, vector<int> test_vect, char *cp)
{
    NSTRUC *np;
    //process file name
    //read infile and create fault list for PI
    for (int k = 0; k < Nnodes; k++)
    {
        np = &Node[k];
        //printf("setting node %d fault list to undefined\n", np->num);
        np->val = Undefine; //initial each nodes val
        for (int l = 0; l < MAXFAULT; l++)
        {
            np->DFS_List[l].SSaF = -1;
            np->DFS_List[l].Dnode = NULL;
        }
        np->DFS_List[0].Dnode = np;
    }
    for (int p = 0; p < p_in.size(); p++)
    {
        for (int k = 0; k < Nnodes; k++) //for all inputs
        {
            np = &Node[k];
            if (np->num == p_in[p])
            {
                np->val = test_vect[p];
                if (DEBUG_LUKE) printf("assigned value for PI %d to %d\n", np->num, np->val);
                if (DEBUG_LUKE) printf("building fault list for PI %d\n", np->num);
                np->DFS_List[0].Dnode = np;
                //Total_DFS_List[list_count].DFS_List[0].Dnode = np;
                if (np->val == 0)
                    np->DFS_List[0].SSaF = 1;
                //Total_DFS_List[list_count].DFS_List[0].SSaF = 1;
                else
                    np->DFS_List[0].SSaF = 0;
                //Total_DFS_List[list_count].DFS_List[0].SSaF = 0;
                //list_count++;
                //break;
            }
        }
    }
    for (int j = 0; j < Nnodes; j++)
    {
        np = &Node[j];
        //printf("Fault list for line %dSS@%d\n", np->DFS_List[0].Dnode->num,
        //np->DFS_List[0].SSaF);
    }
    for (int k = 0; k < Nnodes; k++)
    {
        np = &Node[k];
        if (DEBUG_LUKE) printf("Node %d has value %d\n", np->num, np->val);
    }
    if (DEBUG_LUKE) printf("---------------RUNNING LOGIC SIM------------------- \n");
    logicsim_single(p_in, test_vect, cp);
	

    for (int l = 0; l < Nnodes; l++)
    {
        np = &Node[l];
        //printf("%d has %d fins\n", np->num, np->fin);
    }
	
	for(int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		np->DFS_List[0].Dnode = np;
		if (np->val == 0)
			np->DFS_List[0].SSaF = 1;
		if (np->val == 1)
			np->DFS_List[0].SSaF = 0;
	}
	
	
	

    int level = 1;
    int detect;
    if (DEBUG_LUKE) printf("max level is: %0d\n", maxlevel);
    for (level; level <= maxlevel; level++)
    {
        if (DEBUG_LUKE) printf("level = %d\n", level);
        for (int i = 0; i < Nnodes; i++)
        {
            int c_val_count;
            np = &Node[i];
            if (DEBUG_LUKE) printf("node = %d\n", np->num);

            //check if node in question has upstream fault lists defined
            detect = 1;
            for (int j = 0; j < np->fin; j++) //each input
            {
                if (DEBUG_LUKE) printf("traversing inputs\n");
                if (np->unodes[j]->DFS_List[0].SSaF < 0)
                {
                    detect = 0;
                    //printf("node %d has upstream node %d undefined\n", np->num, np->unodes[j]->num);
                }
                else
                {
                    //printf("node %d has upstream node %d defined\n", np->num, np->unodes[j]->num);
                }
            }

            np = &Node[i];
            if (np->level == level && detect)
            {
                if (DEBUG_LUKE) printf("node %d has upstream nodes defined. Building fault list\n", np->num);
                int c_count = 0;
                int count_one = 0;
                int c_val;
                int i_val;      //controlling val and inverting val for each gate
                int nc_val = 1; //flag for if all inputs hold non-controling value
                if (DEBUG_LUKE) printf("evaulating line %d that has %d gate upstream\n", np->num, np->type);
                switch (np->type)
                {
                case BRCH:
                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;
                    list_union(np);
                    break;
                case XNOR: //todo
                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;
                    list_union(np);
                    break;
                case XOR:
                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;

                    if (DEBUG_LUKE) printf("All values for XOR gate are non-controlling. \n");
                    //check if all inputs hold non-controlling values
                    if (DEBUG_LUKE) printf("All inputs hold non-controling value for AND gate. Combining lists of inputs. \n");
                    list_union(np); //combine lists of inputs into list of gate output node
                    break;
                case OR:
                    c_val = 1;
                    i_val = 0;

                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;

                    if (DEBUG_LUKE) printf("Controlling value of OR gate is 1. \n");

                    c_val_count = 0;

                    //check if all inputs hold non-controlling values

                    for (int i = 0; i < np->fin; i++)
                    {

                        if (DEBUG_LUKE) printf("inputs %d has value %d\n", np->unodes[i]->num, np->unodes[i]->val);
                        if (np->unodes[i]->val == c_val)
                        {
                            c_val_count++;
                        }
                    }
                    if (c_val_count == 0)
                    {
                        if (DEBUG_LUKE) printf("All inputs hold non-controling value for AND gate. Combining lists of inputs. \n");
                        list_union(np); //combine lists of inputs into list of gate output node
                    }
                    else if (c_val_count == np->fin) //all inputs are controlling
                    {
                        if (DEBUG_LUKE) printf("both intputs hold Controling value. if more than 2 inputs, we need to fix this\n");
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if(DEBUG_LUKE) printf("Fault %dSS@%d is in fault list of line %d\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF, np->num);
                            k++;
                        }
                    }
                    else
                    {
                        if (DEBUG_LUKE) printf("Inputs are a combination of controlling and non-controlling values\n");
                        list_combination(np, c_val);
                        if (DEBUG_LUKE) printf("Fault list for line %d: \n", np->num);
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF);
                            k++;
                        }
                    }
                    break;
                case NOR:
                    c_val = 1;
                    i_val = 1;

                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;

                    if (DEBUG_LUKE) printf("Controlling value of OR gate is 1. \n");

                    c_val_count = 0;

                    //check if all inputs hold non-controlling values

                    for (int i = 0; i < np->fin; i++)
                    {
                        if (DEBUG_LUKE) printf("inputs %d has value %d\n", np->unodes[i]->num, np->unodes[i]->val);

                        if (np->unodes[i]->val == c_val)
                        {
                            c_val_count++;
                        }
                    }
                    if (c_val_count == 0)
                    {
                        if (DEBUG_LUKE) printf("All inputs hold non-controling value for AND gate. Combining lists of inputs. \n");
                        list_union(np); //combine lists of inputs into list of gate output node
                    }
                    else if (c_val_count == np->fin) //all inputs are controlling
                    {
                        if (DEBUG_LUKE) printf("both intputs hold Controling value. if more than 2 inputs, we need to fix this\n");
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list of line %d\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF, np->num);
                            k++;
                        }
                    }
                    else
                    {
                        if (DEBUG_LUKE) printf("Inputs are a combination of controlling and non-controlling values\n");
                        list_combination(np, c_val);
                        if (DEBUG_LUKE) printf("Fault list for line %d: \n", np->num);
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF);
                            k++;
                        }
                    }
                    break;
                case NOT:
                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;
                    list_union(np); //combine lists of inputs into list of gate output node
                    break;
                case NAND:
                    c_val = 0;
                    i_val = 1;

                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;

                    if (DEBUG_LUKE) printf("Controlling value of NAND gate is 1. \n");

                    c_val_count = 0;
                    //check if all inputs hold non-controlling values
                    for (int i = 0; i < np->fin; i++)
                    {

                        if (DEBUG_LUKE) printf("inputs %d has value %d\n", np->unodes[i]->num, np->unodes[i]->val);

                        if (np->unodes[i]->val == c_val)
                        {
                            c_val_count++;
                        }
                    }
                    if (c_val_count == 0)
                    {
                        if (DEBUG_LUKE) printf("All inputs hold non-controling value for AND gate. Combining lists of inputs. \n");
                        list_union(np); //combine lists of inputs into list of gate output node
                    }
                    else if (c_val_count == np->fin) //all inputs are controlling
                    {
                        if (DEBUG_LUKE) printf("all intputs hold Controling value.\n");
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list of line %d\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF, np->num);
                            k++;
                        }
                    }
					else if ((c_val_count == np->fin) && (np->fin == 1)) //all inputs are controlling
                    {
                        if (DEBUG_LUKE) printf("all intputs hold Controling value in single input NAND.\n");
                        list_union(np); //combine lists of inputs into list of gate output node
                    }
                    else
                    {
                        if (DEBUG_LUKE) printf("Inputs are a combination of controlling and non-controlling values\n");
                        list_combination(np, c_val);
                        if (DEBUG_LUKE) printf("Fault list for line %d: \n", np->num);
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF);
                            k++;
                        }
                    }
                    break;
                case AND:
                    c_val = 0;
                    i_val = 0;

                    if (np->val == 0)
                        np->DFS_List[0].SSaF = 1;
                    if (np->val == 1)
                        np->DFS_List[0].SSaF = 0;

                    if (DEBUG_LUKE) printf("Controlling value of AND gate is 0. \n");

                    c_val_count = 0;

                    //check if all inputs hold non-controlling values

                    for (int i = 0; i < np->fin; i++)
                    {

                        if (DEBUG_LUKE) printf("inputs %d has value %d\n", np->unodes[i]->num, np->unodes[i]->val);

                        if (np->unodes[i]->val == c_val)
                        {
                            c_val_count++;
                        }
                    }

                    if (c_val_count == 0)
                    {
                        if (DEBUG_LUKE) printf("All inputs hold non-controling value for AND gate. Combining lists of inputs. \n");
                        list_union(np); //combine lists of inputs into list of gate output node
                    }
					else if ((c_val_count == np->fin) && (np->fin == 1)) //all inputs are controlling
                    {
                        if (DEBUG_LUKE) printf("all intputs hold Controling value in single input NAND.\n");
                        list_union(np); //combine lists of inputs into list of gate output node
                    }
                    else if (c_val_count == np->fin) //all inputs are controlling
                    {
                        if (DEBUG_LUKE) printf("all intputs hold Controling value.\n");
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list of line %d\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF, np->num);
                            k++;
                        }
                    }
                    else
                    {
                        if (DEBUG_LUKE) printf("Inputs are a combination of controlling and non-controlling values\n");
                        list_combination(np, c_val);
                        if (DEBUG_LUKE) printf("Fault list for line %d: \n", np->num);
                        int k = 0;
                        while (np->DFS_List[k].SSaF != -1)
                        {
                            if (DEBUG_LUKE) printf("Fault %dSS@%d is in fault list\n", np->DFS_List[k].Dnode->num, np->DFS_List[k].SSaF);
                            k++;
                        }
                    }
                    break;
                default:
                    if (DEBUG_LUKE) printf("No found gate type\n");
                }
            }
				
        }
    }
	
	

    //combine PO faults into single fault list
    DFS_Node DFS_Total[MAXFAULT]; //hold all faults detected

    for (int i = 0; i < MAXFAULT; i++) //clear Dnode of intersection list
    {
        DFS_Total[i].Dnode = NULL;
		DFS_Total[i].SSaF = -1;
    }

    int k = 0;
    while (Poutput[0]->DFS_List[k].Dnode != NULL)
    {
        DFS_Total[k] = Poutput[0]->DFS_List[k];
        k++;
    }
    int duplicate = 0;
    for (int l = 1; l < Npo; l++) //for all po
    {
        int m = 0;
        while (Poutput[l]->DFS_List[m].Dnode != NULL) //for all faults at po
        {
            int n = 0;
            while (DFS_Total[n].Dnode != NULL)
            {
                if ((Poutput[l]->DFS_List[m].Dnode == DFS_Total[n].Dnode) || (Poutput[l]->DFS_List[m].SSaF == -1))
                {
                    duplicate = 1;
                }
                n++;
            }
            if (duplicate == 0)
            {
                DFS_Total[k] = Poutput[l]->DFS_List[m]; //add fault to list;
                if (DEBUG_LUKE) printf("Adding fault %dSS@%d to total fault list\n", Poutput[l]->DFS_List[m].Dnode->num, Poutput[l]->DFS_List[m].SSaF);
                k++;
            }
            m++;
            duplicate = 0;
        }
    }
    int ndx = 0;
	
	for(int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		//if (np->fout == 0)
		//{
			if (DEBUG_LUKE) std::cout << "PO " << np->num << " has fault list " << np->DFS_List[0].Dnode->num << "@" << np->DFS_List[0].SSaF << std::endl;
			if (DEBUG_LUKE) std::cout << "PO " << np->num << " has value " << np->val << std::endl;
		//}
	}
    vector<string> flist;
    string f_str;
	std::cout << "Max level is " << maxlevel << std::endl;

    while ((DFS_Total[ndx].Dnode != NULL) && (DFS_Total[ndx].SSaF != -1))
    {
        std::stringstream line_stream;
        std::stringstream fault_stream;
        line_stream << DFS_Total[ndx].Dnode->num;
        string line_string(line_stream.str());
        fault_stream << DFS_Total[ndx].SSaF;
        string fault_string(fault_stream.str());
        f_str = line_string + "@" + fault_string;
        flist.push_back(f_str);
        if (DEBUG_LUKE) std::cout << flist[ndx] << std::endl;
        ndx++;
    }
    return flist;
}

//--------------------------------DFS--------------------------------

int dfs(char *cp)
{
    if (DEBUG_LUKE) printf("running dfs\n");
    //process file name
    char testpattern[200] = {};
    char faultlist[200] = {};
    char outputfile[200] = {};
    NSTRUC *np;
    int maxlev;
    int g = 0;
    int i;
    for (i = 1; i < 100; i++)
    {
        if (line[i] == ' ')
            break;
        else
        {
            testpattern[g] = line[i];
            g++;
        }
    }
    i++;
    g = 0;
    for (i; i < 100; i++)
    {
        if (line[i] == '\n')
            break;
        else
        {
            outputfile[g] = line[i];
            g++;
        }
    }
    //process filename end

    //read test vector
    vector<int> Pin;
    vector< vector<int> > Pvalue;
    ifstream infile1(testpattern, ios::in);

    if (!infile1)
        cout << "File " << testpattern << " does not exist!" << endl;
    else
    {
        char tmp[1024];
        infile1.getline(tmp, sizeof(tmp));
        int i;
        string pp = "";
        for (i = 0; i < sizeof(tmp); i++)
        {
            if (tmp[i] == '\0')
            {
                Pin.push_back(atoi(pp.c_str()));
                pp = "";
                break;
            }
            else if (tmp[i] != ',')
                pp.push_back(tmp[i]); //initial input pin number
            else
            {
                Pin.push_back(atoi(pp.c_str()));
                pp = "";
            }
        }
        int j = 0;
        while (infile1.getline(tmp, sizeof(tmp)))
        {
            Pvalue.push_back({});
            for (i = 0; i < sizeof(tmp); i++)
            {
                if (tmp[i] == '\0')
                    break;
                else
                {
                    if (tmp[i] != ',')
                        Pvalue[j].push_back((tmp[i] - '0')); //initial all coherence input pin value
                }
            }
            j++;
        }
    }
    infile1.close();

	
	if (DEBUG_LUKE) 
	for (std::string::size_type p = 0; p < Pin.size(); p++)
	{
		std::cout << "Input file has primary input: " << Pin[p] << std::endl;
	}
    for (int i = 0; i < Pvalue.size(); i++)
    {
        std::cout << "Vector " << i << " is : " << std::endl;
        for (int j = 0; j < Pvalue[i].size(); j++)
            std::cout << Pvalue[i][j] << " ";
        std::cout << std::endl;
    }
	

	

    vector< vector<string> > fault_results;
    for (int i = 0; i < Pvalue.size(); i++)
    {
        fault_results.push_back(dfs_single(Pin, Pvalue[i], cp)); //should be i
    }

    for (int i = 0; i < fault_results.size(); i++)
    {
        std::cout << "Resulting Fault List or Vector " << i << " is : " << std::endl;
        for (int j = 0; j < fault_results[i].size(); j++)
            std::cout << fault_results[i][j] << " ";
        std::cout << std::endl;
    }

    //add first fault list to final fault vector
    for (int i = 0; i < fault_results[0].size(); i++)
    {
        Faults_Covered.push_back(fault_results[0][i]);
    }
    int duplicate = 0;
    for (int i = 1; i < fault_results.size(); i++) //for each fault vector
    {
        for (int j = 0; j < fault_results[i].size(); j++) //for each fault in vector
        {
            duplicate = 0;
            for (int k = 0; k < Faults_Covered.size(); k++) //for each prevously covered fault
            {
                if (fault_results[i][j] == Faults_Covered[k])
                {
                    duplicate = 1;
                    std::cout << "Fault " << fault_results[i][j] << "is a duplicate" << std::endl;
                }
            }
            if (duplicate == 0)
                Faults_Covered.push_back(fault_results[i][j]);
        }
    }
	std::cout << "Number of faults covered is " << Faults_Covered.size() << std::endl;
	
	std::cout << "Output file is " << outputfile << std::endl;
	if (output_results)
	{
		ofstream myfile(outputfile);

		for (int i = 0; i < Faults_Covered.size(); i++)
		{
			std::cout << Faults_Covered[i] << std::endl;
			myfile << Faults_Covered[i] << "\n";
		}
		myfile.close();
	}
}

int rtg(char *cp)
{
    //dissect input arguemnts to data structures
    printf("running dfs\n");
    //primary inputs list generation
    std::string tp_report;
    std::string fc_out;
    std::string n_total_string;
    std::string n_tfcr_string;

    int n_total; //the number of total random test patterns to generate
    int n_tfcr;  //thee frequency of FC repor

    int scan_val;
    NSTRUC *np;

    //----------MODIFY FOR 4 INPUT ARGUMENTS--------//
    //process file name

    for (scan_val = 1; scan_val < 100; scan_val++)
    { //for n_total
        if (line[scan_val] == ' ')
            break;
        else
        {
            n_total_string.push_back(line[scan_val]);
        }
    }
    scan_val++;
    for (scan_val; scan_val < 100; scan_val++)
    {
        if (line[scan_val] == ' ')
            break;
        else
        {
            n_tfcr_string.push_back(line[scan_val]);
        }
    }
    scan_val++;
    for (scan_val; scan_val < 100; scan_val++)
    {
        if (line[scan_val] == ' ')
            break;
        else
        {
            tp_report.push_back(line[scan_val]);
        }
    }
    scan_val++;
    for (scan_val; scan_val < 100; scan_val++)
    {
        if (line[scan_val] == '\n')
            break;
        else
        {
            fc_out.push_back(line[scan_val]);
        }
    }
    printf("n_total is %s \n", n_total_string.c_str());
    printf("n_tfcr name is %s \n", n_tfcr_string.c_str());
    printf("test pattern file name is %s \n", tp_report.c_str());
    printf("fault coverage file name is %s \n", fc_out.c_str());

    // object from the class stringstream
    std::stringstream n_total_stream(n_total_string);
    std::stringstream n_tfcr_stream(n_tfcr_string);

    // The object has the value 12345 and stream
    // it to the integer x
    n_total_stream >> n_total;
    n_tfcr_stream >> n_tfcr;
    std::cout << "Value of n_total is: " << n_total << " and value of n_tfcr is: " << n_tfcr << std::endl;
    //----------END MODIFY FOR 4 INPUT ARGUMENTS--------//

    vector<int> pri_in; //primary input vector to be filled by referencing Nodes in base struct;
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->type == 0)
            pri_in.push_back(np->num);
    }
    for (int i = 0; i < pri_in.size(); i++)
    {
        std::cout << pri_in[i] << " is primary input." << std::endl;
    }

    //calculate total faults in circuit
    int tot_faults = Nnodes * 2;
    float cur_fc;
    vector<float> FC;
    //add pi to pri_in;

    vector< vector<int> > test_vectors; //test vectors (vector of vectors)
    vector<int> current_vector;

    srand(time(NULL));
    for (int i = 0; i < n_total; i++)
    {
        for (int j = 0; j < pri_in.size(); j++) //pri_in.size()
        {
            int rand_bit = rand() % 2;
            current_vector.push_back(rand_bit);
        }
        test_vectors.push_back(current_vector);
        current_vector.clear();
    }

    //printing test vector list
    for (int i = 0; i < test_vectors.size(); i++)
    {
        std::cout << "Vector " << i << " is : " << std::endl;
        for (int j = 0; j < test_vectors[i].size(); j++)
            std::cout << test_vectors[i][j] << " ";
        std::cout << std::endl;
    }

    vector< vector<string> > fault_results;
    for (int l = 0; l < test_vectors.size(); l++)
    {
        fault_results.push_back(dfs_single(pri_in, test_vectors[l], cp)); //should be i

        for (int i = 0; i < fault_results.size(); i++)
        {
            if (DEBUG_LUKE) std::cout << "Resulting Fault List or Vector " << i << " is : " << std::endl;
            for (int j = 0; j < fault_results[i].size(); j++)
                if (DEBUG_LUKE) std::cout << fault_results[i][j] << " ";
            std::cout << std::endl;
        }
        int duplicate = 0;
        for (int i = 0; i < fault_results.size(); i++) //for each fault vector
        {
            for (int j = 0; j < fault_results[i].size(); j++) //for each fault in vector
            {
                duplicate = 0;
                for (int k = 0; k < RTG_Faults_Covered.size(); k++) //for each prevously covered fault
                {
                    if (fault_results[i][j] == RTG_Faults_Covered[k])
                    {
                        duplicate = 1;
                        if (DEBUG_LUKE) std::cout << "Fault " << fault_results[i][j] << "is a duplicate" << std::endl;
                    }
                }
                if (duplicate == 0)
                    RTG_Faults_Covered.push_back(fault_results[i][j]);
            }
        }
        cur_fc = ((float)RTG_Faults_Covered.size() / (float)tot_faults) * 100.0;
        std::cout << "i % n_tfcr is " << l % n_tfcr << std::endl;
        if (l % n_tfcr == 0) //10 % 2 == 0?
        {
            FC.push_back(cur_fc);
            std::cout << "pushing FC" << std::endl;
        }
        for (int i = 0; i < RTG_Faults_Covered.size(); i++)
        {
            std::cout << RTG_Faults_Covered[i] << std::endl;
        }
        std::cout << std::setprecision(2) << std::fixed;
        std::cout << "Fault coverage at test pattern " << l << " is " << cur_fc << std::endl;
    }
    for (int i = 0; i < FC.size(); i++)
    {
        std::cout << FC[i] << std::endl;
    }

    //reporting test patterns
    ofstream myfile(tp_report.c_str());
    if (myfile.is_open())
    {
        for (int i = 0; i < pri_in.size(); i++)
        {
            if (i == pri_in.size() - 1)
                myfile << pri_in[i];
            else
                myfile << pri_in[i] << ",";
        }
        myfile << "\n";
        for (int i = 0; i < test_vectors.size(); i++)
        {
            for (int j = 0; j < test_vectors[i].size(); j++)
            {
                if (j == test_vectors[i].size() - 1)
                    myfile << test_vectors[i][j];
                else
                    myfile << test_vectors[i][j] << ",";
            }
            myfile << "\n";
        }
        myfile.close();
    }
    else
        std::cout << "Unable to open file" << std::endl;

    //report fault coverage
    ofstream myfile2(fc_out.c_str());
    if (myfile2.is_open())
    {
        for (int i = 0; i < FC.size(); i++)
        {
            myfile2 << std::fixed << std::setprecision(2) << FC[i];
            myfile2 << "\n";
        }
        myfile2.close();
    }
    else
        std::cout << "Unable to open file" << std::endl;
}
//---------------------------DFS end---------------------------------

//~~~~~~~~~~~~~~~~PFS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pro_valp(NSTRUC *np, vector<int> faultname, vector<int> faultvalue, int counter, int record,int size)
{   int i;
    int unsigned_length = sizeof(unsigned int) * 8; 
    int actuallsize;
    if (record + 1 < size)
        if (counter + 1 <= unsigned_length)
            actuallsize = counter + 1;
        else
            actuallsize = unsigned_length;
    else
        actuallsize = counter - record * unsigned_length + 1;
    
       
    //int int_num = (faultvalue.size() - 1) / (unsigned_length - 1) + 1;
    for(i=record*(unsigned_length-1);i<record*(unsigned_length-1)+actuallsize-1;i++){
        
        if(np->num==faultname[i]){
            int position=actuallsize-2+record*(unsigned_length-1)-i;


            if(faultvalue[i]==1)
                np->valp=np->valp | (1<<position) ;  //corespondence bit change to 1         
            else
                np->valp=np->valp & (~(1<<position)) ;  //corespondence bit change to 0
        }
    }
}

int itoa_bin(unsigned int data, char *str)
{
	if(str == NULL)
		return -1;
	
	char *start = str;
 
	while(data)
	{
		if(data & 0x1)
			*str++ = 0x31;
		else
			*str++ = 0x30;
 
		data >>= 1;
	}
 
	*str = 0;
	
	//reverse the order
	char *low, *high, temp;
	low = start, high = str - 1;
	
	while(low < high)
	{
		temp = *low;
		*low = *high;
		*high = temp;
 
		++low; 
		--high;
	}
 
	return 0;
}

int pfs(char *cp)
{

    //process file name
    char testpattern[200] = {};
    char faultlist[200] = {};
    char outputfile[200] = {};
    NSTRUC *np;
    int maxlev;
    int g = 0;
    int i;
    for (i = 1; i < 100; i++)
    {
        if (line[i] == ' ')
            break;
        else
        {
            testpattern[g] = line[i];
            g++;
        }
    }
    i++;
    g = 0;
    for (i; i < 100; i++)
    {
        if (line[i] == ' ')
            break;
        else
        {
            faultlist[g] = line[i];
            g++;
        }
    }
    i++;
    g = 0;
    for (i; i < 100; i++)
    {
        if (line[i] == '\n')
            break;
        else
        {
            outputfile[g] = line[i];
            g++;
        }
    }
    //process filename end

    //read test vector
    vector<int> Pin;
    vector< vector<int> > Pvalue;
    ifstream infile1(testpattern, ios::in);

    if (!infile1)
        cout << "File " << testpattern << " does not exist!" << endl;
    else
    {
        char tmp[1024];
        infile1.getline(tmp, sizeof(tmp));
        int i;
        string pp = "";
        for (i = 0; i < sizeof(tmp); i++)
        {
            if (tmp[i] == '\0')
            {
                Pin.push_back(atoi(pp.c_str()));
                pp = "";
                break;
            }
            else if (tmp[i] != ',')
                pp.push_back(tmp[i]); //initial input pin number
            else
            {
                Pin.push_back(atoi(pp.c_str()));
                pp = "";
            }
        }
        int j = 0;
        while (infile1.getline(tmp, sizeof(tmp)))
        {
            Pvalue.push_back({});
            for (i = 0; i < sizeof(tmp); i++)
            {
                if (tmp[i] == '\0')
                    break;
                else
                {
                    if (tmp[i] != ',')
                        Pvalue[j].push_back((tmp[i] - '0')); //initial all coherence input pin value
                }
            }
            j++;
        }
    }
    infile1.close();
    //read test vector end

    //read faultlist
    vector<int> faultname;
    vector<int> faultvalue;
    ifstream infile2(faultlist, ios::in);
    if (!infile2)
        cout << "File " << faultlist << " does not exist!" << endl;
    else
    {
        char tmp[1024];

        int i;

        while (infile2.getline(tmp, sizeof(tmp)))
        {
            int j = 0;
            string pp = "";
            for (i = 0; i < sizeof(tmp); i++)
            {
                if (tmp[i] == '\0')
                    break;
                else
                {

                    if (tmp[i] == '@')
                    {
                        faultname.push_back(atoi(pp.c_str()));
                        j++;
                    }
                    else if (j == 0)
                    {
                        pp.push_back((tmp[i])); //initial all coherence input pin value
                        //cout<<tmp[i]<<endl;
                        //cout<<tmp[i] - '0'<<endl;
                    }
                    else
                    {
                        faultvalue.push_back((tmp[i] - '0'));
                    }
                }
            }
        }
    }
    infile2.close();
    //read faultlist end



    //Get the maximum level of the circuit
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        np->level = Undefine; //initial each nodes' level
        np->initial_sign = false;   //initial each nodes' val
    }
    int k = 1;
    while (k)
    {
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (((np->level) < 0) && (np->type == IPT))
                np->level = 0;
            else
            {
                if (np->level < 0)
                {
                    int in = 0, plev = 0;
                    int j;
                    for (j = 0; j < np->fin; j++)
                    {
                        if (np->unodes[j]->level >= 0)
                        {
                            in++;
                            if (np->unodes[j]->level > plev)
                                plev = np->unodes[j]->level;
                        }
                    }
                    if (in == np->fin && (np->type == BRCH || np->type == XOR || np->type == OR || np->type == NOR || np->type == NOT || np->type == NAND || np->type == AND || np->type == XOR || np->type == XNOR))
                        np->level = plev + 1;
                    if (np->level > maxlev)
                        maxlev = np->level;
                }
            }
        }
        k = 0;
        for (i = 0; i < Nnodes; i++) //To determine whether to exit the while loop
        {
            np = &Node[i];
            if (np->level < 0)
                k++;
        }
    }
    //maximum-circuit-level get ends



    //get system word bit size
    int unsigned_length = sizeof(unsigned int) * 8;    
    int int_num = (faultvalue.size() - 1) / (unsigned_length - 1) + 1;
    //get system word bit size end



    //core
    ofstream outfile(outputfile);//ready output file
    vector<int> tmp_result_n;
    vector<int> tmp_result_v;
    for (i = 0; i < Pvalue.size(); i++)
    { //each i repersent each single pfs

        //generate word
        vector< vector<unsigned int> > Word(Pin.size(), vector<unsigned int>(int_num));
        cout << Word.size() <<"*"<< endl;
        cout << Word[1].size() <<"*"<< endl;
        int j;
        int counter = 0;
        for (j = 0; j < faultname.size(); j++)
        {
            int k;
            if (counter % (unsigned_length) == 0)
                j--;
            
            for (k = 0; k < Pin.size(); k++)
            {
                if (counter % (unsigned_length) == 0)
                {
                    Word[k][counter / (unsigned_length)] = Pvalue[i][k];
                }
                else if (faultname[j] == Pin[k])
                {
                    Word[k][counter / (unsigned_length)] = (Word[k][counter / (unsigned_length)] << 1) + faultvalue[j];
                }
                else
                {
                    Word[k][counter / (unsigned_length)] = (Word[k][counter / (unsigned_length)] << 1) + Pvalue[i][k];
                }
            }
            counter++;
        }
        //generate word end


        

        //begin to propagate        
        int record; counter--;        
        for(record=0;record<Word[0].size();record++){
            //initial each node value

            for (i = 0; i < Nnodes; i++)
            {
                np = &Node[i];
                np->initial_sign=false; //initial each nodes' val sign
                np->valp=0;
            }

            int cc = 0;
            for (i = 0; i < Pin.size(); i++)
            {
                int j;
                for (j = 0; j < Nnodes; j++)
                {
                    np = &Node[j];
                    if (np->num == Pin[i])
                    {
                        np->valp = Word[i][record]; //put test vector's each line value to corespondent node
                        np->initial_sign=true;
                        cc++;
                        break;
                    }
                }
            }
            int level = 1;
            int j;
            int detect;
            //printf("max level is: %0d", maxlev);
            for (level; level <= maxlev; level++)
            {
                for (i = 0; i < Nnodes; i++)
                {
                    np = &Node[i];

                    for (j = 0; j < np->fin; j++)
                    {
                        detect = 1;
                        if (np->unodes[j]->initial_sign ==false )
                        {
                            detect = 0;
                        }
                    }

                    np = &Node[i];
                    if (np->level == level && detect)
                    {
                        int c_count = 0;
                        int count_one = 0;
                        int count_x = 0;
                        switch (np->type)
                        {
                        case BRCH: 
                            np->valp = np->unodes[0]->valp;
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case XNOR: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=~((np->valp)^(np->unodes[c_count]->valp));                                                                   
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case XOR: 

                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)^(np->unodes[c_count]->valp);
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case OR: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)|(np->unodes[c_count]->valp);
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case NOR: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)|(np->unodes[c_count]->valp);
                            }
                            np->valp=~(np->valp);
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case NOT: 
                            np->valp=~np->valp;
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case NAND: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)&(np->unodes[c_count]->valp);
                            }
                            np->valp=~(np->valp);
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case AND: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)&(np->unodes[c_count]->valp);
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        default:
                            printf("No found gate type\n");
                        }
                    }
                }
            }

            //looking for SSF position
            vector<unsigned int> resulta;//store all might fault value of each outpin
            //vector<unsigned int> resultr;//store each outpin good value
            for (j = 0; j < Nnodes; j++)
            {
                np = &Node[j];
                if (np->dnodes[0] == NULL)
                    resulta.push_back(np->valp);
            }
            string str[resulta.size()];
            for(j=0;j<resulta.size();j++){
                char tmp[200]={};                
                itoa_bin(resulta[j],tmp);
                str[j]=tmp;

                //compensate unsigned loss
                int actuallsize;
                if(record+1<Word[0].size())
                    if(counter+1<=unsigned_length)
                        actuallsize=counter+1;
                    else
                        actuallsize=unsigned_length;
                else
                    actuallsize=counter-record*unsigned_length+1;
                int kk=1;
                while(kk){
                    if(str[j].size()<actuallsize)
                        str[j]='0'+str[j];
                    else
                        kk=0;
                }


                int ll;
                for(ll=1;ll<str[j].size();ll++){
                    if(str[j][0]!=str[j][ll]){
                        
                        if(tmp_result_n.size()==0){
                            tmp_result_n.push_back(faultname[record*(unsigned_length-1)+ll-1]);
                            tmp_result_v.push_back(faultvalue[record*(unsigned_length-1)+ll-1]);
                        }
                        else{
                            int mm;
                            int nn=tmp_result_n.size();
                            int sign=1;
                            for(mm=0;mm<nn;mm++){
                                if((tmp_result_n[mm]==faultname[record*(unsigned_length-1)+ll-1])
                                && (tmp_result_v[mm]==faultvalue[record*(unsigned_length-1)+ll-1]))
                                sign=0;
                            }        
                            if(sign){
                                tmp_result_n.push_back(faultname[record*(unsigned_length-1)+ll-1]);  
                                tmp_result_v.push_back(faultvalue[record*(unsigned_length-1)+ll-1]);
                            }                              
                            
                        }

                        
                    }
                }
            }


            //release key variable
            
            vector<unsigned int>().swap(resulta);
            
            
                    


        }
        //propagate end        
        vector<vector<unsigned int> >().swap(Word);
    }
    for(i=0;i<tmp_result_n.size();i++){
        outfile<<tmp_result_n[i]<<"@"<<tmp_result_v[i]<<endl;
    }
    outfile.close();
    vector<int>().swap(tmp_result_n);
    vector<int>().swap(tmp_result_v);

}


//~~~~~~~~~~~PFS end ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~podem~~~~~~~~~~~~~~~~


//begine podem non-core variable and function definition 
vector<NSTRUC*> DF_podem;

int test_np;
bool test_np_value;
static int podem_fail = 0;
int dynamnic_node;
int dynamnic_value;
char namefile[MAXLINE];
bool compare(char* a)
{
	if (a[0] == 'a') { if (a[1] == 't') { if (a[2] == 'p') { if (a[3] == 'g') { printf("NEXT \n");  return true; } } } }
	return false;
}

bool control_bit(int tp){
   switch(tp) {
      case 0: return(0);
      case 1: return(0);
      case 2: return(1);
      case 3: return(1);
      case 4: return(1);
      case 5: return(0);
      case 6: return(0);
      case 7: return(0);
   }
   return 0;
}

bool inversion(int tp){
   switch(tp) {
      case 0: return(0);
      case 1: return(0);
      case 2: return(0);
      case 3: return(0);
      case 4: return(1);
      case 5: return(1);
      case 6: return(1);
      case 7: return(0);
   }
   return 0;
}

int lev_modify(char *kk){
/*
    queue<NSTRUC*> q;
    int i,j,max;
    NSTRUC *np;
    char *gname();
	char namefile_temp[80]="";
    int level_max=0;

    level_max++;
    for(i=0;i<Nnodes;i++){
      np = &Node[i];
      if (np->fin == 0){
         np->level = 0;
         lev_array.resize(level_max);
         lev_array[0].push_back(np);
         q.push(np);
      }
      np->fin_lev = np->fin;
    }   

    while(!q.empty()){
         level_max++;
         lev_array.resize(level_max);
         int s = q.size();
         for (i=0;i<s;i++){
            int max = 0;
            NSTRUC *temp = q.front();
            q.pop();
            for(j = 0; j < temp->fout; j++) {
               np = temp->dnodes[j];
               np->fin_lev--;
               if (np->fin_lev == 0){
                  q.push(np); 
                  lev_array[level_max-1].push_back(np);                  
               }
            }
            for(j = 0; j < temp->fin; j++) {
               np = temp->unodes[j];
               if (max < np->level)
                  max = np->level;
            }
            if (temp->fin!=0)           
               temp->level = max +1;
         }
    }



    i=0;
    strcpy(namefile_temp, title);
	while (namefile_temp[i]!='.'){
       i++;
    }
    namefile_temp[i] = '\0';
    strcat(namefile_temp,"_level.txt\0\n"); 
    FILE *fp;
    fp = fopen(namefile_temp,"w+");
    for(i = 0; i<Nnodes; i++) {
      np = &Node[i];
      fprintf(fp,"%d,%d\n", np->num, np->level);
    }
    fclose(fp);
	memset(namefile_temp, '\0', sizeof(namefile_temp));
    return 0;
*/



    int Ngate = 0;
    int g = 0;
    char m[100] = {};
    int i;
    for (i = 1; i < 100; i++)
    {
        if (line[i] == '\n')
            break;
        else
        {
            m[g] = line[i];
            g++;
        }
    }
    
    FILE *fpp;
    strcpy(m, title);
    strcat(m,"_level.txt\0\n"); 
    fpp = fopen(m, "w");
    NSTRUC *np;
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        np->level = Undefine;
        if (np->type != IPT && np->type != BRCH)
            Ngate++;
    }
    int k = 1;
    while (k)
    {
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (((np->level) < 0) && (np->type == IPT))
                np->level = 0;
            else
            {
                if (np->level < 0)
                {
                    int in = 0, plev = 0;
                    int j;
                    for (j = 0; j < np->fin; j++)
                    {
                        if (np->unodes[j]->level >= 0)
                        {
                            in++;
                            if (np->unodes[j]->level > plev)
                                plev = np->unodes[j]->level;
                        }
                    }
                    if (in == np->fin && (np->type == BRCH || np->type == XOR || np->type == OR || np->type == NOR || np->type == NOT || np->type == NAND || np->type == AND || np->type == XNOR))
                    {
                        np->level = plev + 1;
                        if (np->level > maxlevel)
                            maxlevel = np->level;
                    }
                }
            }
        }

        k = 0;
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (np->level < 0)
                k++;
        }
    }
    lev_array.resize(maxlevel+1);
//vector< vector<NSTRUC*> > lev_array;
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        np->fin_lev = np->fin;
        lev_array[np->level].push_back(np);
        fprintf(fpp, "%d %d\n", np->num, Node[i].level);
    }
    fclose(fpp);

    

    return 0;





}

int imply(NSTRUC* kv_node, bool kv_value_v, bool kv_value_u){
	NSTRUC* np;
	int j;
	bool temp1,temp2;
	int DF_count;
	for (int i = 0; i < Nnodes; i++) {
		np = &Node[i];
		if (np->num == kv_node->num){
			np->Vval = kv_value_v;
			np->Uval = kv_value_u;
			if (np->num!= test_np){
			np->Vval_fault = kv_value_v;
			np->Uval_fault = kv_value_u;}				
			//printf("imply: np->Vval = %d, np->Uval =%d\n", np->Vval, np->Uval);					
		}
	}

   	for (int level_x=1;level_x<lev_array.size();level_x++){
           //cout<<"level_x"<<":"<<level_x<<"  lev_array.size():"<<lev_array.size()<<endl;
      	for (int level_num=0;level_num<lev_array[level_x].size();level_num++)
      	{   //cout<<"level_num:"<<level_num<<"  lev_array[level_x].size()"<<lev_array[level_x].size()<<endl;
         	np=lev_array[level_x][level_num];
         	switch (np->type)
				{
					case 1://BRANCH
									for (j=0; j<(np->fin); j++)
									{
										np->Vval = np->unodes[j]->Vval;
										np->Uval = np->unodes[j]->Uval;
										np->Vval_fault = np->unodes[j]->Vval_fault;
										np->Uval_fault = np->unodes[j]->Uval_fault;
									}
									break;
					case 2://XOR
									np->Vval = 0;
									np->Uval = 0;
									np->Vval_fault = 0;
									np->Uval_fault = 0;
									for (j=0; j<(np->fin); j++)
									{	
										if (np->unodes[j]->Vval==0 && np->unodes[j]->Uval==1){
											np->Vval = 0;
											np->Uval = 1;
											np->Vval_fault = 0;
											np->Uval_fault = 1;
										}
										else
										{
											np->Vval = (np->Vval)^(np->unodes[j]->Vval);
											np->Uval = (np->Uval)^(np->unodes[j]->Uval);
											np->Vval_fault = (np->Vval_fault)^(np->unodes[j]->Vval_fault);
											np->Uval_fault = (np->Uval_fault)^(np->unodes[j]->Uval_fault);									
										}
									}
									break;
					case 3://OR
									np->Vval = 0;
									np->Uval = 0;
									np->Vval_fault = 0;
									np->Uval_fault = 0;
									for (j=0; j<(np->fin); j++)
									{
										np->Vval = (np->Vval)|(np->unodes[j]->Vval);
										np->Uval = (np->Uval)|(np->unodes[j]->Uval);
										np->Vval_fault = (np->Vval_fault)|(np->unodes[j]->Vval_fault);
										np->Uval_fault = (np->Uval_fault)|(np->unodes[j]->Uval_fault);
									}
									break;
					case 4://NOR
									np->Vval = 0;
									np->Uval = 0;
									np->Vval_fault = 0;
									np->Uval_fault = 0;
									for (j=0; j<(np->fin); j++)
									{
										np->Vval = (np->Vval)|(np->unodes[j]->Vval);
										np->Uval = (np->Uval)|(np->unodes[j]->Uval);
										np->Vval_fault = (np->Vval_fault)|(np->unodes[j]->Vval_fault);
										np->Uval_fault = (np->Uval_fault)|(np->unodes[j]->Uval_fault);
									}
									temp1 = !np->Uval;
									temp2 = !np->Vval;
									np->Vval = temp1;
									np->Uval = temp2;
									temp1 = !np->Uval_fault;
									temp2 = !np->Vval_fault;
									np->Vval_fault = temp1;
									np->Uval_fault = temp2;
									break;
					case 5://NOT
									for (j=0; j<(np->fin); j++)
									{
										np->Vval = !np->unodes[j]->Uval;
										np->Uval = !np->unodes[j]->Vval;
										np->Vval_fault = !np->unodes[j]->Uval_fault;
										np->Uval_fault = !np->unodes[j]->Vval_fault;
									}
									break;
					case 6://NAND
									np->Vval = 1;
									np->Uval = 1;
									np->Vval_fault = 1;
									np->Uval_fault = 1;
									for (j=0; j<(np->fin); j++)
									{
										np->Vval = (np->Vval)&(np->unodes[j]->Vval);
										np->Uval = (np->Uval)&(np->unodes[j]->Uval);
										np->Vval_fault = (np->Vval_fault)&(np->unodes[j]->Vval_fault);
										np->Uval_fault = (np->Uval_fault)&(np->unodes[j]->Uval_fault);
									}
									temp1 = !np->Uval;
									temp2 = !np->Vval;
									np->Vval = temp1;
									np->Uval = temp2;
									temp1 = !np->Uval_fault;
									temp2 = !np->Vval_fault;
									np->Vval_fault = temp1;
									np->Uval_fault = temp2;
									break;
					case 7://AND
									np->Vval = 1;
									np->Uval = 1;
									np->Vval_fault = 1;
									np->Uval_fault = 1;
									for (j=0; j<(np->fin); j++)
									{
										np->Vval = (np->Vval)&(np->unodes[j]->Vval);
										np->Uval = (np->Uval)&(np->unodes[j]->Uval);
										np->Vval_fault = (np->Vval_fault)&(np->unodes[j]->Vval_fault);
										np->Uval_fault = (np->Uval_fault)&(np->unodes[j]->Uval_fault);
									}
									break;
					default:		printf("error\n");
				}
				if (((np->Vval==0)&&(np->Uval==1))||((np->Vval_fault==0)&&(np->Uval_fault==1))){
					np->Vval = 0;
					np->Uval = 1;
					np->Vval_fault = 0;
					np->Uval_fault = 1;				
				}
				if (np->num == test_np){
					np->Vval_fault = test_np_value;
					np->Uval_fault = test_np_value;	
				}


      	}
   	}

	DF_count = 0;
	for (int i = 0; i < Nnodes; i++) {
		np = &Node[i];
		if ((np->Vval!=np->Vval_fault)&&(np->Uval!=np->Uval_fault)){
			for (j=0; j<(np->fout); j++){
				if (((np->dnodes[j]->Vval==0)&&(np->dnodes[j]->Uval==1))||((np->dnodes[j]->Vval_fault==0)&&(np->dnodes[j]->Uval_fault==1))){
					DF_podem.push_back(np->dnodes[j]);
					//printf ("add DF\n");
					DF_count++;
				}
			}
		}	
	} // create DF

	//printf("i have done imply\n");
	return DF_count;
}

int PODEM_prepare()
{   /*
	NSTRUC* np;
	for (int i = 0; i < Nnodes; i++) { 
		np = &Node[i];
		np->Vval = 0;
		np->Uval = 1;
        
		if (np->num == test_np){
			np->Vval_fault = test_np_value;
			np->Uval_fault = test_np_value;	
		}
		else 
		{
			np->Vval_fault = 0;
			np->Uval_fault = 1;
		}
			
	} //assign X to every node
	DF_podem.clear();
	return 0;
    */
    NSTRUC* np;
	for (int i = 0; i < Nnodes; i++) { 
		np = &Node[i];
		np->Vval = 0;
		np->Uval = 1;
        np->Vval_fault = 0;
		np->Uval_fault = 1;
		if (np->num == test_np){
			np->Vval_fault = test_np_value;
			np->Uval_fault = test_np_value;	
		}
		
			
	} //assign X to every node
	DF_podem.clear();
	return 0;
}

int objective(vector<NSTRUC* >& kv_node, vector<bool>& kv_value)
{
	NSTRUC* np;
	for (int i = 0; i < Nnodes; i++) {
		np = &Node[i];
		if (np->num == test_np){	
			if (np->Vval==0 && np->Uval==1)
			{
				kv_node[0] = np;
				kv_value[0] = !test_np_value;
				//printf("return i, vbar in objective\n");
				return 1;
			}
		}
	} // create fault value for D or D bar

	//printf("DF_podem.size() = %d\n",DF_podem.size());
	for (int i=0; i<DF_podem.size();i++){
		np = DF_podem[i];
		for (int j=0; j<(np->fin); j++){
			//printf("%d, %d, %d\n",np->unodes[j]->num,np->unodes[j]->Vval,np->unodes[j]->Uval);
			if (((np->unodes[j]->Vval==0)&&(np->unodes[j]->Uval==1))||((np->unodes[j]->Vval_fault==0)&&(np->unodes[j]->Uval_fault==1))){
				kv_node[0] = np->unodes[j];
				kv_value[0] = !control_bit(np->unodes[j]->type);
				//printf("return j,cbar in objective\n");
				return 1;
			}
		}
	}//return j, cbar
	//printf("objective error\n");
	return 0;
}

int backtrace(vector<NSTRUC* >& kv_node, vector<bool>& kv_value)
{
	NSTRUC* np;
	bool value;
	bool inver;
	np = kv_node[0];
	value = kv_value[0];
	while (np->type!=0){
		inver = inversion(np->type);
		for (int i=0; i<(np->fin); i++){
			if ((np->unodes[i]->Vval==0)&&(np->unodes[i]->Uval==1)){
				value = value^inver;
				np = np->unodes[i];
				break;//revise
				//printf("np->type=%d\n",np->type);
			}
		}	
	}
	kv_node[0] = np;
	kv_value[0] = value;
	//printf("run backtrace, np->num = %d, np->value = %d\n",np->num,value);
	return 0;
}

int PODEM_run(){
	NSTRUC* np;
	vector<NSTRUC* > kv_node(1,NULL);
	vector<bool> kv_value(1,0);
	NSTRUC* j;
	int DF_count;
	bool value;
	int flag;

	//printf("podem run 1 times\n");
	for(int i=0; i<Npo; i++)
	{
		np = Poutput[i];
		if ((np->Vval != np->Vval_fault)&&(np->Uval != np->Uval_fault))
		{
			//printf("error at PO\n");
			return 1;
		}
	}

	for (int i = 0; i < Nnodes; i++) {
		np = &Node[i];
		if ((np->num == test_np)&&((np->Vval!=0)||(np->Uval!=1))){ //this node value is not unknown
			if ((np->Vval != (!test_np_value))&&(np->Uval != (!test_np_value))){
				//printf("test not possible\n");
				return 0;
			}
		}
	}//test not possible
	flag = 1;
	flag = objective(kv_node,kv_value);   //****
	if (flag==0){
		podem_fail = 1;	
		return 0;
	}
	backtrace(kv_node,kv_value);          //****
	j = kv_node[0];
	value = kv_value[0];
	DF_count = imply(j,value,value);
	//printf("first time\n");
	if (PODEM_run() == 1) 
        return 1;

	for (int i = 0; i < Nnodes; i++) { 
		np = &Node[i];
		if (np->type != 0){
			np->Vval = 0;
			np->Uval = 1;
			if (np->num!= test_np){
			np->Vval_fault = 0;
			np->Uval_fault = 1;}	
		}	
	} //assign X to every node
	for (int ii=0;ii<DF_count;ii++){
		DF_podem.pop_back();
	}
	DF_count = imply(j,!value,!value);	  //****
	//printf("second time\n");
	if (PODEM_run() == 1) 
        return 1;

	for (int i = 0; i < Nnodes; i++) { 
		np = &Node[i];
		if (np->type != 0){
			np->Vval = 0;
			np->Uval = 1;
			if (np->num!= test_np){
			np->Vval_fault = 0;
			np->Uval_fault = 1;}
		}	
	} //assign X to every node

	for (int ii=0;ii<DF_count;ii++){
		DF_podem.pop_back();
	}
	imply(j,0,1);
	//printf("Exit PODEM run\n");
	return 0;
}


int podem(char* cp)
{	
	NSTRUC* np;
	char value;
    //Get fault node number and corresponse fault value
	string num_value = "";
	string fault_value = "";
	//char namefile_temp[MAXLINE];
	{
        int j=0;
        for(j;j<10;j++){
            if(line[j+1]!=' '){
                num_value.push_back(line[j+1]);
            }
            else{
                j++;
                fault_value.push_back(line[j+1]);
                break;
            }
        }
    }
    
	stringstream tmpstr(num_value);
    tmpstr>>test_np;
    tmpstr.clear();
    //tmpstr(fault_value);
    test_np_value=fault_value[0]-'0';
/*
	if (!compare(num_value)){
		test_np = num_value[0]-'0';
		test_np_value = fault_value[0]-'0';
	}
	else
	{
		test_np = dynamnic_node;
		test_np_value = dynamnic_value;		
	}
*/	
   
    lev_modify(cp);    
	PODEM_prepare();
	PODEM_run();	
	
	//printf ("test input is success = %d\n",flag);
	//for (int i = 0; i < Nnodes; i++) {
	//	np = &Node[i];
	//	printf("%d, %d, %d, %d, %d\n", np->num,np->Vval,np->Uval,np->Vval_fault,np->Uval_fault);
	//}

	for(int i=0; i<Npi; i++)
	{
		np = Pinput[i];
		if ((np->Vval==0)&&(np->Uval==0))
			value = '0';
		else if ((np->Vval==1)&&(np->Uval==1))
			value = '1';
		else
			value = 'X';
		
		printf("%d,%c\n",np->num, value);
	}

	if (podem_fail)
		printf("cannot find input test pattern\n");
	else
		printf("find the input test pattern\n");	
    
    //generate outputfile name
    char outputfile[100]={};
    string tmpfilename = "";
    int i=0;
    for(i;i<100;i++){
        if(title[i]!='\0')
            tmpfilename.push_back(title[i]);
        
        else{
            tmpfilename=tmpfilename+"_PODEM_"+num_value+"@"+fault_value+".txt";
            
            break;
            
        }

    }
    strcpy(outputfile, tmpfilename.c_str());
    //output file name end

    //print final value to file
    FILE *fp;
    fp = fopen(outputfile,"w+");
	for(int i=0; i<Npi; i++)
	{
		np = Pinput[i];
		if ((np->Vval==0)&&(np->Uval==0))
			value = '0';
		else if ((np->Vval==1)&&(np->Uval==1))
			value = '1';
		else
			value = 'X';
		
		fprintf(fp,"%d,%c\n",np->num, value);
	}
    fclose(fp);    
	memset(outputfile, '\0', sizeof(outputfile));
    //print final value to file end
    return 0;

}

//~~~~~~~~~~~~~~~~~~~~~~~podem end~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~ATPG_DET~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef struct faultliststruct{
    int ndnum;
    int ndval;
} faultnodeinfo;
 


vector<faultnodeinfo> faultlisf;
vector<faultnodeinfo> podemout;
vector<faultnodeinfo> dalgout;
vector<faultnodeinfo> pfsout;



int creatfaultlist(){
    NSTRUC *np;
    int i=0;
    faultnodeinfo m,n;
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        m.ndnum=np->num;
        n.ndnum=np->num;
        m.ndval=0;
        n.ndval=1;
        faultlisf.push_back(m);
        faultlisf.push_back(n);
    
    }
    return 0;
}


bool call_podem(int n, char *tmp){
    NSTRUC* np;
    char value;
    //Get fault node number and corresponse fault value
	string num_value = "";
	string fault_value = "";

	test_np=faultlisf[n].ndnum;
    test_np_value=faultlisf[n].ndval;
	
   
    lev_modify(tmp);    
	PODEM_prepare();
	PODEM_run();	
	
	//printf ("test input is success = %d\n",flag);
	//for (int i = 0; i < Nnodes; i++) {
	//	np = &Node[i];
	//	printf("%d, %d, %d, %d, %d\n", np->num,np->Vval,np->Uval,np->Vval_fault,np->Uval_fault);
	//}

	for(int i=0; i<Npi; i++)
	{
		np = Pinput[i];
		if ((np->Vval==0)&&(np->Uval==0))
			value = '0';
		else if ((np->Vval==1)&&(np->Uval==1))
			value = '1';
		else
			value = 'X';
		
		//printf("%d,%c\n",np->num, value);
	}

	
    
    

    //print final value to file
    faultnodeinfo k;
    if(podem_fail){
        printf("cannot find input test pattern\n");
        return false;
    }
    else{
        printf("find the input test pattern\n");
        for(int i=0; i<Npi; i++){	    
            np = Pinput[i];
            if ((np->Vval==0)&&(np->Uval==0))
                value = '0';
            else if ((np->Vval==1)&&(np->Uval==1))
                value = '1';
            else
                value = '0';
            k.ndnum=np->num;
            k.ndval=value-'0';
            podemout.push_back(k);
            //fprintf(fp,"%d,%c\n",np->num, value);
        }
        return true;
    }
	
    
   
    
}


int dalg_run(int node_num);
int dalg(char* cp);
bool call_dalg(int n, char *tmp){
    std::cout << "Running D-Algorithm on circuit." << std::endl;
    NSTRUC* np;
    faultnodeinfo temp_val;
    //Line " 1 0"
   
    char tmp_str[200]={};                
    //itoa_bin(faultlisf[n].ndnum,tmp_str);
    
/*
    stringstream tmpstr(faultlisf[n].ndnum);
    tmpstr>>test_np;
    tmpstr.clear();
    */
   stringstream linestream;
   stringstream faultstream;
   linestream << faultlisf[n].ndnum;
   faultstream << faultlisf[n].ndval;
   
   string line_string(linestream.str());
   string fault_string(faultstream.str());
    
    line[0] = ' ';
    int lineval = 1;
    int j = 0;
    while (line_string[j] != '\0')
    {
        line[lineval] = line_string[j];
        lineval++;
        j++;
    }
    //for (lineval = 1; lineval < fnode.size() + 1; lineval++)
        //line[lineval] = fnode[lineval - 1];
    line[lineval] = ' ';
    lineval++;
    line[lineval] = fault_string[0];
    lineval++;
    line[lineval] = '\000';

	if (dalg(tmp)) //failed
	{
		std::cout << "FAILURE. CANNOT BE DONE" <<std::endl;
        return false;
	}
    else
    {
        for (int j = 0; j < Nnodes; j++)
			{
				np = &Node[j];
				if (np->type == IPT)
				{	if (np->five_val == D_bar  || np->five_val == ZERO)	
					{
						temp_val.ndnum = np->num;
                        temp_val.ndval = 0;
                        dalgout.push_back(temp_val);
						if (DEBUG_LUKE) std::cout  << np->num <<",0" << std::endl;
					}
					else if (np->five_val == D  || np->five_val == ONE)
					{
						temp_val.ndnum = np->num;
                        temp_val.ndval = 1;
                        dalgout.push_back(temp_val);
						if (DEBUG_LUKE) std::cout << np->num << ",1" << std::endl;
					}
					else
					{
						int randnum = rand() % 2;
						temp_val.ndnum = np->num;
                        temp_val.ndval = randnum;
                        dalgout.push_back(temp_val);
						if (DEBUG_LUKE) std::cout << np->num << "," << randnum << std::endl;
					}
				}
			}
	    return true;
    }
}

int call_pfs(string str){
    //process file name
    
    NSTRUC *np;
    int maxlev=0;
    

    //read test vector
    vector<int> Pin;
    vector< vector<int> > Pvalue;
    Pvalue.push_back({});
    int i=0;

    if((!strcmp(str.c_str(),"PODEM")||(!strcmp(str.c_str(),"podem")))){
        for(i=0;i<podemout.size();i++){
            Pin.push_back(podemout[i].ndnum);
            Pvalue[0].push_back(podemout[i].ndval);
        }  
    }
    else{
        for(i=0;i<dalgout.size();i++){
            Pin.push_back(dalgout[i].ndnum);
            Pvalue[0].push_back(dalgout[i].ndval);
        }  
    } 
    //read test vector end

    //read faultlist
    vector<int> faultname;
    vector<int> faultvalue;
    for(i=0;i<faultlisf.size();i++){
        faultname.push_back(faultlisf[i].ndnum);
        faultvalue.push_back(faultlisf[i].ndval);

    }    
    //read faultlist end



    //Get the maximum level of the circuit
    for (i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        np->level = Undefine; //initial each nodes' level
        np->initial_sign = false;   //initial each nodes' val
        np->valp=0;
    }
    int k = 1;
    while (k)
    {
        for (i = 0; i < Nnodes; i++)
        {
            np = &Node[i];
            if (((np->level) < 0) && (np->type == IPT))
                np->level = 0;
            else
            {
                if (np->level < 0)
                {
                    int in = 0, plev = 0;
                    int j;
                    for (j = 0; j < np->fin; j++)
                    {
                        if (np->unodes[j]->level >= 0)
                        {
                            in++;
                            if (np->unodes[j]->level > plev)
                                plev = np->unodes[j]->level;
                        }
                    }
                    if (in == np->fin && (np->type == BRCH || np->type == XOR || np->type == OR || np->type == NOR || np->type == NOT || np->type == NAND || np->type == AND || np->type == XOR || np->type == XNOR))
                        np->level = plev + 1;
                    if (np->level > maxlev)
                        maxlev = np->level;
                }
            }
        }
        k = 0;
        for (i = 0; i < Nnodes; i++) //To determine whether to exit the while loop
        {
            np = &Node[i];
            if (np->level < 0)
                k++;
        }
    }
    //maximum-circuit-level get ends



    //get system word bit size
    int unsigned_length = sizeof(unsigned int) * 8;    
    int int_num = (faultvalue.size() - 1) / (unsigned_length - 1) + 1;
    //get system word bit size end



    //core
    
    vector<int> tmp_result_n;
    vector<int> tmp_result_v;
    for (i = 0; i < Pvalue.size(); i++)
    { //each i repersent each single pfs

        //generate word
        vector< vector<unsigned int> > Word(Pin.size(), vector<unsigned int>(int_num));
        cout << Word.size() <<"*"<< endl;
        cout << Word[1].size() <<"*"<< endl;
        int j;
        int counter = 0;
        for (j = 0; j < faultname.size(); j++)
        {
            int k;
            if (counter % (unsigned_length) == 0)
                j--;

            for (k = 0; k < Pin.size(); k++)
            {
                if (counter % (unsigned_length) == 0)
                {
                    Word[k][counter / (unsigned_length)] = Pvalue[i][k];
                }
                else if (faultname[j] == Pin[k])
                {
                    Word[k][counter / (unsigned_length)] = (Word[k][counter / (unsigned_length)] << 1) + faultvalue[j];
                }
                else
                {
                    Word[k][counter / (unsigned_length)] = (Word[k][counter / (unsigned_length)] << 1) + Pvalue[i][k];
                }
            }
            counter++;
        }
        //generate word end


        

        //begin to propagate        
        int record; counter--;        
        for(record=0;record<Word[0].size();record++){
            //initial each node value
            int cc = 0;
            for (cc = 0; cc < Nnodes; cc++)
            {
                np = &Node[cc];
                np->initial_sign=false; //initial each nodes' val sign
                np->valp=0;
            }
            cc=0;
            int ii;
            for (ii = 0; ii < Pin.size(); ii++)
            {
                int j;
                for (j = 0; j < Nnodes; j++)
                {
                    np = &Node[j];
                    if (np->num == Pin[ii])
                    {
                        np->valp = Word[ii][record]; //put test vector's each line value to corespondent node
                        np->initial_sign=true;
                        cc++;
                        break;
                    }
                }
            }
            int level = 1;
            int j;
            int detect;
            
            //printf("max level is: %0d", maxlev);
            for (level; level <= maxlev; level++)
            {   int jj;
                for (jj = 0; jj < Nnodes; jj++)
                {  

                    np = &Node[jj];

                    int check_point;
                    if(np->type!=IPT){
                        for (check_point = 0; check_point < np->fin; check_point++)
                        {
                            detect = 1;
                        
                            if (np->unodes[check_point]->initial_sign ==false )
                            {
                                detect = 0;
                                break;
                            }                        
                        }
                    }
                    //np = &Node[jj];
                    if (np->level == level && detect)
                    {
                        int c_count = 0;
                        int count_one = 0;
                        int count_x = 0;
                        switch (np->type)
                        {
                        case BRCH: 
                            np->valp = np->unodes[0]->valp;
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case XNOR: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=~((np->valp)^(np->unodes[c_count]->valp));                                                                   
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case XOR: 

                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)^(np->unodes[c_count]->valp);
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case OR: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)|(np->unodes[c_count]->valp);
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case NOR: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)|(np->unodes[c_count]->valp);
                            }
                            np->valp=~(np->valp);
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case NOT: 
                            np->valp=~np->valp;
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case NAND: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)&(np->unodes[c_count]->valp);
                            }
                            np->valp=~(np->valp);
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        case AND: 
                            for (c_count = 0; c_count < np->fin; c_count++)
                            {
                                if (c_count == 0)
                                    np->valp=np->unodes[c_count]->valp;
                                else
                                    np->valp=(np->valp)&(np->unodes[c_count]->valp);
                            }
                            np->initial_sign=true;
                            pro_valp(np,faultname,faultvalue,counter,record,Word[0].size());
                            break;

                        default:
                            printf("No found gate type\n");
                        }
                    }
                }
            }

            //looking for SSF position
            vector<unsigned int> resulta;//store all might fault value of each outpin
            //vector<unsigned int> resultr;//store each outpin good value
            for (j = 0; j < Nnodes; j++)
            {
                np = &Node[j];
                if (np->dnodes[0] == NULL)
                    resulta.push_back(np->valp);
            }

            string str[resulta.size()];

            for(j=0;j<resulta.size();j++){
                char tmp[200]={};                
                itoa_bin(resulta[j],tmp);
                str[j]=tmp;

                //compensate unsigned loss
                int actuallsize;
                if(record+1<Word[0].size())
                    if(counter+1<=unsigned_length)
                        actuallsize=counter+1;
                    else
                        actuallsize=unsigned_length;
                else
                    actuallsize=counter-record*unsigned_length+1;
                int kk=1;
                while(kk){
                    if(str[j].size()<actuallsize)
                        str[j]='0'+str[j];
                    else
                        kk=0;
                }


                int ll;
                for(ll=1;ll<str[j].size();ll++){
                    if(str[j][0]!=str[j][ll]){
                        
                        if(tmp_result_n.size()==0){
                            tmp_result_n.push_back(faultname[record*(unsigned_length-1)+ll-1]);
                            tmp_result_v.push_back(faultvalue[record*(unsigned_length-1)+ll-1]);
                        }
                        else{
                            int mm;
                            int nn=tmp_result_n.size();
                            int sign=1;
                            for(mm=0;mm<nn;mm++){
                                if((tmp_result_n[mm]==faultname[record*(unsigned_length-1)+ll-1])
                                && (tmp_result_v[mm]==faultvalue[record*(unsigned_length-1)+ll-1]))
                                sign=0;
                            }        
                            if(sign){
                                tmp_result_n.push_back(faultname[record*(unsigned_length-1)+ll-1]);  
                                tmp_result_v.push_back(faultvalue[record*(unsigned_length-1)+ll-1]);
                            }                              
                            
                        }

                        
                    }
                }
            }


            //release key variable
            
            vector<unsigned int>().swap(resulta);
            
            
                    


        }
        //propagate end        
        vector<vector<unsigned int> >().swap(Word);
    }
   






   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    faultnodeinfo kk;
    for(i=0;i<tmp_result_n.size();i++){
        kk.ndnum=tmp_result_n[i];
        kk.ndval=tmp_result_v[i];
        pfsout.push_back(kk);
        //cout<<tmp_result_n[i]<<"@"<<tmp_result_v[i]<<endl;
    }
        
    vector<int>().swap(tmp_result_n);
    vector<int>().swap(tmp_result_v);
    vector<int>().swap(Pin);
    vector< vector<int> >().swap(Pvalue);
    vector<int>().swap(faultname);
    vector<int>().swap(faultvalue);
    return 0;
  
}

int call_dfs(string str, char* cp){
	NSTRUC* np;
	
	 //read test vector
    vector<int> Pin;
    vector<int> Pvalue;
    int i=0;

    if((!strcmp(str.c_str(),"PODEM")||(!strcmp(str.c_str(),"podem")))){
        for(i=0;i<podemout.size();i++){
            Pin.push_back(podemout[i].ndnum);
            Pvalue.push_back(podemout[i].ndval);
        }  
    }
    else{
        for(i=0;i<dalgout.size();i++){
            Pin.push_back(dalgout[i].ndnum);
            Pvalue.push_back(dalgout[i].ndval);
        }  
    } 
	
	vector<string> fault_results;
    fault_results = dfs_single(Pin, Pvalue, cp); //should be i
	
	faultnodeinfo kk;	
	
	string delimiter = "@";

	size_t pos = 0;
	string fnode;
	string ffault;
	
	vector < int > curr_vect;
	vector<int> pri_in; //primary input vector to be filled by referencing Nodes in base struct;
  
	std::cout << "Running ATPG on missing checkpoint faults" << std::endl;
	for (int i = 0; i < fault_results.size(); i++)
	{
		string s = fault_results[i];
		bool f_found = false;
		while ((pos = s.find(delimiter)) != std::string::npos) {
			fnode = s.substr(0, pos);
			if (DEBUG_LUKE) std::cout << fnode << std::endl;
			s.erase(0, pos + delimiter.length());
			break;
		}
		ffault = s;
		if (DEBUG_LUKE) std::cout << ffault << std::endl;
		
		stringstream node_stream(fnode); 
		stringstream fault_stream(ffault); 

		node_stream >> kk.ndnum; 
		fault_stream >> kk.ndval;	
		pfsout.push_back(kk);
	}
	/*
    for(i=0;i<tmp_result_n.size();i++){
        kk.ndnum=tmp_result_n[i];
        kk.ndval=tmp_result_v[i];
        pfsout.push_back(kk);
        //cout<<tmp_result_n[i]<<"@"<<tmp_result_v[i]<<endl;
    }
	*/
	
}


bool reducefaultlist(){
    int beginn=faultlisf.size();
    
    //cout<<"faultlist size "<<faultlisf.size()<<endl;
    int i=0,j=0;
    for(i=0;i<pfsout.size();i++){
        for(j=0;j<faultlisf.size();j++){
            if(pfsout[i].ndval==faultlisf[j].ndval
                && pfsout[i].ndnum==faultlisf[j].ndnum){
                    faultlisf.erase(faultlisf.begin()+j);
                    j=0;
                    break;
                }
                
        }
    }
    int endn=faultlisf.size();
   
    //cout<<"faultlist size "<<faultlisf.size()<<endl;

    
    if(beginn>endn)
        return true;
    else
        return false;
}



int atpg_det(char *cp){
    //timing start
    clock_t starttime,endtime;
    starttime=clock();
	output_results = false;


    //file name processing
    string cktname="",alogr="",outputpattern="",outputreport="";    
    int tmp=1,sign=0,i=0,totalfault=0;
    while(cp[tmp]!='\n'){
        cout<<cp[tmp]<<endl;
        if(cp[tmp]==' ')
            sign=1;
        if(cp[tmp]!=' ' && sign==0){
            cktname.push_back(cp[tmp]);
                   
        }
        else if(cp[tmp]!=' ' && sign==1){
            alogr.push_back(cp[tmp]);
            
        }  
    
        tmp++;
    }
    
    for(i=0;i<cktname.size();i++){
        if(cktname[i]!='.'){
            outputpattern.push_back(cktname[i]);
            outputreport.push_back(cktname[i]);
        }
        else
            break;
        
    }
    if((!strcmp(alogr.c_str(),"PODEM"))||(!strcmp(alogr.c_str(),"podem"))){
        outputpattern=outputpattern+"_PODEM_ATPG_patterns.txt";
        outputreport=outputreport+"_PODEM_ATPG_report.txt";
    }
    else if((!strcmp(alogr.c_str(),"DALG"))||(!strcmp(alogr.c_str(),"dalg"))){
        outputpattern=outputpattern+"_DALG_ATPG_patterns.txt";
        outputreport=outputreport+"_DALG_ATPG_report.txt";
        }
    else{
        cout<<"None Exist Algorithm"<<endl;
        exit(0);
    }
    //file name processing end

    //read circuit and generate all possible SS@F
    char temp[cktname.length()];
    strcpy(temp,cktname.c_str());
    cread(temp);
    creatfaultlist();
    cout<<"faultlist"<<endl;
    for(i=0;i<faultlisf.size();i++){
        cout<<faultlisf[i].ndnum<<" "<<faultlisf[i].ndval<<endl;
    }
    //read circuit and generate all possible SS@F end
   
    
    i=0;sign=0;
    totalfault=faultlisf.size();
    ofstream fppattern,fpreport;
    fppattern.open(outputpattern.c_str());
    bool workn=true;
    
    while(true){
        cout<<"faultlisf.size:"<<faultlisf.size()<<endl;
        if(i>faultlisf.size()||faultlisf.size()==0){
            break;
        }
        if((!strcmp(alogr.c_str(),"PODEM"))||(!strcmp(alogr.c_str(),"podem"))){
            if(call_podem(i,temp)){
                //call_pfs(alogr);     
				call_dfs(alogr, cp);
                workn=reducefaultlist();
                if(workn){
                    i=0;
                    int j;
                    if(sign==0){                
                        for(j=0;j<podemout.size();j++){
                            //output input-node-number to file
                            if(j==podemout.size()-1)
                                fppattern<<podemout[j].ndnum<<endl;                   
                            else
                                fppattern<<podemout[j].ndnum<<',';                                 
                            //output input-node-number to file end                    
                        }
                        for(j=0;j<podemout.size();j++){
                            //output input-node-number correspondent value to file                    
                            if(j==podemout.size()-1)
                                fppattern<<podemout[j].ndval<<endl;                    
                            else
                                fppattern<< podemout[j].ndval<<',';
                            
                            //output input-node-number correspondent value to file
                        }
                        sign=1;
                    }
                    else{
                        
                        for(j=0;j<podemout.size();j++){
                            if(j==podemout.size()-1)
                                fppattern<<podemout[j].ndval<<endl; 
                            else
                                fppattern<<podemout[j].ndval<<',';
                        }
                    }
                }            
                else
                    i++;

                vector<faultnodeinfo>().swap(podemout);
                vector<faultnodeinfo>().swap(pfsout);            
            }        
            else
                i++;
        }
        
        else{
            if(call_dalg(i,temp)){
                //call_pfs(alogr);  
				call_dfs(alogr, cp);				
                workn=reducefaultlist();
                if(workn){
                    i=0;
                    int j;
                    if(sign==0){                
                        for(j=0;j<dalgout.size();j++){
                            //output input-node-number to file
                            if(j==dalgout.size()-1)
                                fppattern<<dalgout[j].ndnum<<endl;                   
                            else
                                fppattern<<dalgout[j].ndnum<<',';                                 
                            //output input-node-number to file end                    
                        }
                        for(j=0;j<dalgout.size();j++){
                            //output input-node-number correspondent value to file                    
                            if(j==dalgout.size()-1)
                                fppattern<<dalgout[j].ndval<<endl;                    
                            else
                                fppattern<< dalgout[j].ndval<<',';
                            
                            //output input-node-number correspondent value to file
                        }
                        sign=1;
                    }
                    else{
                        
                        for(j=0;j<dalgout.size();j++){
                            if(j==dalgout.size()-1)
                                fppattern<<dalgout[j].ndval<<endl; 
                            else
                                fppattern<<dalgout[j].ndval<<',';
                        }
                    }
                }            
                else
                    i++;

                vector<faultnodeinfo>().swap(dalgout);
                vector<faultnodeinfo>().swap(pfsout);            
            }        
            else
                i++;
        }    
        
    }
    //generate report
    fpreport.open(outputreport.c_str());
    fpreport<<"Algorithm:";
    if((!strcmp(alogr.c_str(),"PODEM"))||(!strcmp(alogr.c_str(),"podem"))){
        fpreport<<"PODEM"<<endl;
    }
    else {
        fpreport<<"DALG"<<endl;
    }
    fpreport<<"Circuit:";
    for(i=0;i<cktname.size();i++){
        if(cktname[i]!='.')
            fpreport<<cktname[i];
        else
            break;        
    }
    float coverage=(1-((float)faultlisf.size()/(float)totalfault))*100;
    //cout<<"totalfault:"<<totalfault<<endl<<"remain fault:"<<faultlisf.size()<<endl;
    //cout<<"coverage:"<<coverage<<endl;
    fpreport<<endl<<"Fault Coverage:"<< coverage<<"%"<<endl<<"Time:";
   
    endtime=clock();
    //time tick end  
    double runtime=(double)((double)(endtime-starttime)/CLOCKS_PER_SEC);
    cout<<starttime<<endl;
    cout<<endtime<<endl;
    cout<<runtime<<endl;
    fpreport<<runtime<<" seconds"<<endl;
    
	
	for (int i = 0; i < faultlisf.size(); i++)
	{
		std::cout << "Missing fault " << faultlisf[i].ndnum << "@" << faultlisf[i].ndval << std::endl;
	}
    
    //fpreport<<runtime<<"seconds"<<endl;
    fpreport.close();
    fppattern.close();
    //generate report end
    vector<faultnodeinfo>().swap(faultlisf);
    vector<faultnodeinfo>().swap(podemout);
    vector<faultnodeinfo>().swap(pfsout);
    vector<faultnodeinfo>().swap(dalgout);

    
    return 0;
}


//~~~~~~~~ATPG_DET end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~









//LIFO for imply and check

vector <NSTRUC*> D_frontier;
vector <NSTRUC*> J_frontier;

bool five_val_equal(e_val a, e_val b)
{
	if ((a == D) && (b == D))
		return true;
	else if ((a == D) && (b == ONE))
		return true;
	else if ((a == ONE) && (b == D))
		return true;
	else if ((a == ONE) && (b == ONE))
		return true;
	else if ((a == D_bar) && (b == D_bar))
		return true;
	else if ((a == D_bar) && (b == ZERO))
		return true;
	else if ((a == ZERO) && (b == D_bar))
		return true;
	else if ((a == ZERO) && (b == ZERO))
		return true;
	
	else 
		return false;
}


bool five_val_conflict(e_val a, e_val b)
{
	if ((a == D) && (b == D))
		return true;
	else if ((a == D) && (b == ONE))
		return true;
	else if ((a == ONE) && (b == D))
		return true;
	else if ((a == ONE) && (b == ONE))
		return true;
	else if ((a == D_bar) && (b == D_bar))
		return true;
	else if ((a == D_bar) && (b == ZERO))
		return true;
	else if ((a == ZERO) && (b == D_bar))
		return true;
	else if ((a == ZERO) && (b == ZERO))
		return true;
	else if ((a == X) || (b == X))
		return true;
	else 
		return false;
}

e_val not_five_val(e_val a)
{
	if (a == ONE)
		return ZERO;
	if (a == ZERO)
		return ONE;
	if (a == D)
		return D_bar;
	if (a == D_bar)
		return D;
	else 
		return X;
}

e_val not_five_val_back(e_val a)
{
	if (a == ONE)
		return ZERO;
	if (a == ZERO)
		return ONE;
	if (a == D)
		return ZERO;
	if (a == D_bar)
		return ONE;
	else 
		return X;
}

e_val five_val_back(e_val a)
{
	if (a == ONE)
		return ONE;
	if (a == ZERO)
		return ZERO;
	if (a == D)
		return ONE;
	if (a == D_bar)
		return ZERO;
	else 
		return X;
}

e_val xor_five_val (e_val a, e_val b)
{
	std::cout << "Calculating XOR of " << fval(a) << " and " <<fval(b) << std::endl;
	if ((a == ONE) && (b == ONE))
		return ZERO;
	if ((a == ONE) && (b == ZERO))
		return ONE;
	if ((a == ONE) && (b == D))
		return D_bar;
	if ((a == ONE) && (b == D_bar))
		return D;
	if ((a == ZERO) && (b == ONE))
		return ONE;
	if ((a == ZERO) && (b == ZERO))
		return ZERO;
	if ((a == ZERO) && (b == D))
		return D;
	if ((a == ZERO) && (b == D_bar))
		return D_bar;
	if ((a == D) && (b == ONE))
		return D_bar;
	if ((a == D) && (b == ZERO))
		return D;
	if ((a == D) && (b == D))
		return D_bar;
	if ((a == D) && (b == D_bar))
		return D;
	if ((a == D_bar) && (b == ONE))
		return D;
	if ((a == D_bar) && (b == ZERO))
		return D_bar;
	if ((a == D_bar) && (b == D))
		return D;
	if ((a == D_bar) && (b == D_bar))
		return D_bar;
	else 
		return X;
}


int propegate_five_val(NSTRUC* np)
{
	for (int i = 0; i < np->fout; i++)
	{
		if ((np->dnodes[i]->five_val != D) || (np->dnodes[i]->five_val != D_bar))
		{
			if (np->dnodes[i]->type == NOT)
			{
				np->dnodes[i]->five_val = not_five_val(np->five_val);
				propegate_five_val(np->dnodes[i]);
			}
			else if (np->dnodes[i]->type == BRCH)
			{
				np->dnodes[i]->five_val = np->five_val;
				propegate_five_val(np->dnodes[i]);
			}
		}
	}
}

int propegate_five_val_back(NSTRUC* np)
{
	if (np->type == NOT)
	{
		
		np->unodes[0]->five_val = not_five_val(np->five_val);
		propegate_five_val_back(np->unodes[0]);
	}
	else if (np->type == BRCH)
	{
		np->unodes[0]->five_val = np->five_val;
		propegate_five_val_back(np->unodes[0]);
	}
}

int imply_and_check_recursive(NSTRUC* np)
{
	if (DEBUG_LUKE) std::cout << "Running recursive imply and check on node " << np->num <<std::endl;
	//Push function input signal np and all fan outs into LIFO S (arbitrary order for FO, but np should be at bottom)
	NSTRUC* np_temp;
	vector <NSTRUC*> S;
	
	if(0)
	{
		
		for (int i = 0; i < Nnodes; i++)
			if (Node[i].num == 753)
			{
				np_temp = &Node[i];
				break;
			}
		std::cout << "Node " << np_temp->num << " has value " << fval(np_temp->five_val) << std::endl;
		if (!((np_temp->five_val == D) ||  (np_temp->five_val == D_bar) ||  (np_temp->five_val == X)))
		for (int i = 0; i < 500000000; i++)  //debug delay 
		{
			
		}
	}
	if (0)
	for (int i = 0; i < 500000000; i++)  //debug delay 
	{
		
	}
	/*
	//check if fan-in lines are defined
	for (int i = 0; i < fin; i++)
	{
		np_temp = np->unodes[i];
		if (np_temp->five_val == X)
			return 0;
	}
	//all fan-in lines are defined, now we evaulate
	*/
	bool d_front_needed = true;
	bool d_inputs = false;
	bool any_inputs_NC = true;
	bool all_inputs_NC = true;
	bool all_inputs_def = true;
	switch (np->type)
	{
		case BRCH: 
			if (np->five_val == X)
			{
				//if (!((np->five_val == D) || (np->five_val == D_bar)))
				np->five_val = np->unodes[0]->five_val;
			}
			else if(five_val_equal(np->five_val, np->unodes[0]->five_val))
			{
				//if (((np->five_val == D) || (np->five_val == D_bar)) && ((np->unodes[0]->five_val == ZERO) || (np->unodes[0]->five_val == ONE)))
				//{
					
				//}
				//else
				//{
					//np->five_val = np->unodes[0]->five_val;
				//}
			}
			else if ((np->five_val != X) && np->unodes[0]->five_val == X)
			{
				if (DEBUG_LUKE)	std::cout << "US not defined yet. Will be assigned in DEFINE" << std::endl;
			}
			else 
			{
				if (DEBUG_LUKE) std::cout << "IMPLY AND CHECK FAILED IN FORWARD. TRY NEXT FRONTIER"  << std::endl;
				//assign other branch to node value
				
				//np->five_val = np->dnodes[0]->five_val;
				//propegate_five_val_back(np);
				return 1;
			}
			break;
			

		case XNOR: 
			//check all inputs are defined
			for (int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val == X)
					all_inputs_def = false;
			}
			if (all_inputs_def && (np->five_val != X))
				d_front_needed = false;
			//check if inside D frontier already
			for(int j = 0; j < D_frontier.size(); j++) //for each element in D_frontier
				if (np == D_frontier[j])
				{ //evaluate here for debug
					d_front_needed = false;
					if (DEBUG_LUKE)	std::cout << "Duplicate node "<<  D_frontier[j]->num << " in D-Fronter" << std::endl;
				}
			if (d_front_needed)
			{
				if (DEBUG_LUKE)	std::cout << "Adding node "<< np->num << " to D-Fronter" << std::endl;
				D_frontier.push_back(np);
			}
			break;

		case XOR: 
			//check all inputs are defined
			for (int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val == X)
					all_inputs_def = false;
			}
			if (all_inputs_def && (np->five_val != X))
				d_front_needed = false;
			//check if inside D frontier already
			for(int j = 0; j < D_frontier.size(); j++) //for each element in D_frontier
				if (np == D_frontier[j])
				{ //evaluate here for debug
					d_front_needed = false;
					if (DEBUG_LUKE)	std::cout << "Duplicate node "<<  D_frontier[j]->num << " in D-Fronter" << std::endl;
				}
			if (d_front_needed)
			{
				if (DEBUG_LUKE)	std::cout << "Adding node "<< np->num << " to D-Fronter" << std::endl;
				D_frontier.push_back(np);
			}

			break;

		case OR: 
			//check all inputs for D
			//if all inputs are D or 1, no d_frontier needed
			for (int j = 0; j < np->fin; j++)
			{
				if ((np->unodes[j]->five_val == D) || (np->unodes[j]->five_val == D_bar))
					d_inputs = true;
			}
			if (!d_inputs)
				d_front_needed = false;
			
			
			if (np->five_val == D_bar)
				d_front_needed = false;
			
			//check if any inputs are X
			for (int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val == X)
					all_inputs_def = false;
			}
			if (all_inputs_def)
				d_front_needed = false;
			//check if inside D frontier already
			for(int j = 0; j < D_frontier.size(); j++) //for each element in D_frontier
				if (np == D_frontier[j])
				{
					d_front_needed = false;
					if (DEBUG_LUKE)	std::cout << "Duplicate node "<< np->num << " in D-Fronter" << std::endl;
				}
			if (d_front_needed)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to D-Fronter" << std::endl;
				D_frontier.push_back(np);
			}
			break;

		case NOR: 
			//check all inputs for D or D_bar
			for (int j = 0; j < np->fin; j++)
			{
				if ((np->unodes[j]->five_val == D) || (np->unodes[j]->five_val == D_bar))
					d_inputs = true;
			}
			if (!d_inputs)
				d_front_needed = false;
			//check if any inputs are X
			for (int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val == X)
					all_inputs_def = false;
			}
			if (all_inputs_def)
				d_front_needed = false;
			if (np->five_val == D_bar)
				d_front_needed = false;
			//check if inside D frontier already
			for(int j = 0; j < D_frontier.size(); j++) //for each element in D_frontier
				if (np == D_frontier[j])
				{
					d_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in D-Fronter" << std::endl;
				}
			if (d_front_needed)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to D-Fronter" << std::endl;
				D_frontier.push_back(np);
			}
			break;

		case NOT: 
			if (np->five_val == X)
				np->five_val = not_five_val(np->unodes[0]->five_val);
			else if ((np->five_val == X) && !five_val_conflict(np->unodes[0]->five_val, not_five_val(np->five_val)))
			{
				if (DEBUG_LUKE) std::cout << "IMPLY AND CHECK FAILED IN FORWARDS. CONFLICT ON BRANCH DETECTED. TRY NEXT FRONTIER"  << std::endl;
				if (!((np->five_val == D) || (np->five_val == D_bar)) && ((np->unodes[0]->five_val == ONE) || (np->unodes[0]->five_val == ZERO) || (np->unodes[0]->five_val == X)))	
				{
					np->unodes[0]->five_val = not_five_val_back(np->five_val);
					propegate_five_val_back(np->unodes[0]);
				}
				else
				{
					np->unodes[0]->five_val = not_five_val(np->five_val);
					propegate_five_val(np->unodes[0]);
				}
				return 1;
			}
			break;

		case NAND: 
			//check if any inputs have D or D_bar
			//if no inputs are D or D_bar, no D frontier needed
			for (int j = 0; j < np->fin; j++)
			{
				if ((np->unodes[j]->five_val == D) || (np->unodes[j]->five_val == D_bar))
					d_inputs = true;
			}
			if (!d_inputs)
				d_front_needed = false;
			//check if any inputs are X
			for (int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val == X)
					all_inputs_def = false;
			}
			if (all_inputs_def)
				d_front_needed = false;
			
			if (np->five_val == D)
				d_front_needed = false;
			//check if inside D frontier already
			for(int j = 0; j < D_frontier.size(); j++) //for each element in D_frontier
				if (np == D_frontier[j])
				{
					d_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in D-Fronter" << std::endl;
				}
			if (d_front_needed)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to D-Fronter" << std::endl;
				D_frontier.push_back(np);
			}
			break;

		case AND:		
			if (DEBUG_LUKE) std::cout << "AND Gate in recursive " << std::endl;
			//check all inputs for D
			//if all inputs are D or 1, no d_frontier needed
			/*
			for (int j = 0; j < np->fin; j++)
			{
				if ((np->unodes[j]->five_val == D) || (np->unodes[j]->five_val == D_bar))
					d_inputs = true;
			}
			if (!d_inputs)
				d_front_needed = false;
			*/
			//check if any inputs are X
			for (int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val == X)
					all_inputs_def = false;
			}
			if (all_inputs_def && (np->fin == 1))
			{
				np->five_val = np->unodes[0]->five_val;
				d_front_needed = false;
			}
			else if (all_inputs_def)
			{
				d_front_needed = false;
			}
			
			if (np->five_val == D_bar)
				d_front_needed = false;
			//check if inside D frontier already
			for(int j = 0; j < D_frontier.size(); j++) //for each element in D_frontier
				if (np == D_frontier[j])
				{
					d_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in D-Fronter" << std::endl;
				}
			if (d_front_needed)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to D-Fronter" << std::endl;
				D_frontier.push_back(np);
			}
			break;


		default:
			if (DEBUG_LUKE) printf("No found gate type\n");
	}	
	if (np->five_val != X)	
		for (int i = 0; i < np->fout; i++)
		{
			np_temp = np->dnodes[i];
			if (DEBUG_LUKE) std::cout << "Adding node " << np_temp->num << " to S" << std::endl;
			S.push_back(np_temp);
		}
	//while (S != empty)
		//pop line L value from stack
		//if (US nodes defined)
			//evaluate line L
			//recursively call imply_and_check(Line L)
	while (S.size() != 0)
	{
		np_temp = S[S.size()-1];
		S.pop_back();
		imply_and_check_recursive(np_temp);
		
	}
	//S is empty, np was just popped into line L
	//Return SUCCESS
	return 0;
}

int imply_and_check_define(NSTRUC* np)
{
	if (DEBUG_LUKE) std::cout << "Running define imply and check on node " << np->num << " which is a " << gname(np->type) << std::endl;
	//Push function input signal np and all fan outs into LIFO S (arbitrary order for FO, but np should be at bottom)
	NSTRUC* np_temp;
	vector <NSTRUC*> S;
	if(0)
	{
		
		for (int i = 0; i < Nnodes; i++)
			if (Node[i].num == 479)
			{
				np_temp = &Node[i];
				break;
			}
		std::cout << "Node " << np_temp->num << " has value " << fval(np_temp->five_val) << std::endl;
		if (!((np_temp->five_val == D) ||  (np_temp->five_val == D_bar) ||  (np_temp->five_val == X)))
		for (int i = 0; i < 500000000; i++)  //debug delay 
		{
			
		}
	}
	/*
	//check if fan-in lines are defined
	for (int i = 0; i < fin; i++)
	{
		np_temp = np->unodes[i];
		if (np_temp->five_val == X)
			return 0;
	}
	//all fan-in lines are defined, now we evaulate
	*/
	//if (error not at PO)
	bool j_front_needed = true;
	e_val c_val_out;
	e_val d_val_out;
	bool is_def = false;
	bool branch_conflict = false;
	switch (np->type)
	{
		case BRCH: 			
			if ((np->unodes[0]->five_val != X) && !five_val_equal(np->unodes[0]->five_val, np->five_val))
			{
				if (!((np ->five_val == D) || (np ->five_val == D_bar)))
					propegate_five_val(np->unodes[0]);
				else
				{
					for (int i = 0; i < np->unodes[0]->fout; i++)
					{
						if(!((np->unodes[0]->dnodes[i]->five_val == D) || (np->unodes[0]->dnodes[i]->five_val == D_bar)))
							np->unodes[0]->dnodes[i]->five_val = five_val_back(np->five_val);
					}
				}
					//propegate_five_val(np);
				return 1;
			}
			//check if branch base is different
			
			
			//if upstream node is defined, set to current node
			/*
			if ((np->unodes[0]->five_val != X) && !five_val_equal(np->five_val, np->unodes[0]->five_val) )
			{
				if (DEBUG_LUKE) std::cout << "IMPLY AND CHECK FAILED IN BACKWARDS. TRY NEXT FRONTIER"  << std::endl;
				//if (!((np ->five_val == D) || (np ->five_val == D_bar)))
				//{
					//np->five_val = np->unodes[0]->five_val;
					//propegate_five_val(np);
					return 1;
				//}
			}
			*/
			//else
			//{
			//if ((!(np->unodes[0]->five_val == D) || (np->unodes[0]->five_val == D_bar)))
			//if (!((np->five_val == D) || (np->five_val == D_bar)))\
		
			if (np->unodes[0]->five_val == X && ((np->five_val == D) || (np->five_val == D_bar)))
			{
				np->unodes[0]->five_val = five_val_back(np->five_val);
				for (int i = 0; i < np->unodes[0]->fout; i++)
				{
					if(!((np->unodes[0]->dnodes[i]->five_val == D) || (np->unodes[0]->dnodes[i]->five_val == D_bar)))
						np->unodes[0]->dnodes[i]->five_val = five_val_back(np->five_val);
				}
			}
			else if(np->unodes[0]->five_val == X )
				np->unodes[0]->five_val = np->five_val;
			
		
		
			else if (((np->five_val == D) || (np->five_val == D_bar)) && ((np->unodes[0]->five_val == ZERO) || (np->unodes[0]->five_val == ONE)))
			{
				if (!five_val_equal(np->five_val, np->unodes[0]->five_val))
					return 1;
			}
			else
			{
				if(!((np->unodes[0]->five_val == D) || (np->unodes[0]->five_val == D_bar)) && five_val_equal(np->five_val, np->unodes[0]->five_val )  && np->five_val != X)
					np->unodes[0]->five_val = five_val_back(np->five_val);
				for (int i = 0; i < np->unodes[0]->fout; i++)
				{
					if(!((np->five_val == D) || (np->five_val == D_bar)) && np->five_val != X)
						if (!((np->unodes[0]->dnodes[i]->five_val == D) || (np->unodes[0]->dnodes[i]->five_val == D_bar)))
							np->unodes[0]->dnodes[i]->five_val = five_val_back(np->five_val);
				}
			}
			/*
			if (DEBUG_LUKE) std::cout << "Setting US node " << np->unodes[0]->num << " to " << fval(np->unodes[0]->five_val) << std::endl;
			for (int i = 0; i < np->unodes[0]->fout; i++)
			{
				if (!((np->unodes[0]->dnodes[i]->five_val == D) || (np->unodes[0]->dnodes[i]->five_val == D_bar)))
					np->unodes[0]->dnodes[i]->five_val = five_val_back(np->unodes[0]->five_val);
			}
			if (DEBUG_LUKE) std::cout << "US node " << np->unodes[0]->num << " now has val " << fval(np->unodes[0]->five_val) << std::endl;
			*/
			//check if in J_Frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np->unodes[0] == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			
			if (np->unodes[0]->fin != 0 && np->five_val != X && j_front_needed) //if not PI, then j frontier needed
				J_frontier.push_back(np->unodes[0]);
			//}
			break;

		case XNOR: 
			if (np->fin == 0)
				j_front_needed = false;
			//check if inside J frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			if (j_front_needed && np->five_val != X)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to J-Fronter in DEFINE" << std::endl;
				J_frontier.push_back(np);
			}
			break;

		case XOR: 
			
			if (np->fin == 0)
				j_front_needed = false;
			//check if inside J frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			if (j_front_needed && np->five_val != X)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to J-Fronter in DEFINE" << std::endl;
				J_frontier.push_back(np);
			}
			break;

		case OR: 
			c_val_out = ONE;
			d_val_out = D;
			//no j frontier required if input
			if (np->fin == 0)
				j_front_needed = false;
			if (np->five_val == D_bar)
			{
				//assign all inputs to ZERO
				for (int i=0; i < np->fin; i++)
				{
					np_temp = np->unodes[i];
					if (np_temp->five_val == X)
					{
						np_temp->five_val = ZERO;
						J_frontier.push_back(np_temp);
					}
					j_front_needed = false;
				}
			}
			//check if inside J frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			for(int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val != X)
					is_def = true;
			}
			if (np->fout == 0 && is_def)
				j_front_needed = false;
			for (int k = 0; k < np->fin; k++)
			{
				if ((np->five_val == D) && ((np->unodes[k]->five_val == D_bar) || (np->unodes[k]->five_val == ZERO)))
					j_front_needed = false;
			}
			if (j_front_needed && np->five_val != X)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to J-Fronter in DEFINE" << std::endl;
				J_frontier.push_back(np);
			}
			break;

		case NOR: 
			c_val_out = ZERO;
			d_val_out = D_bar;
			//no j frontier required if input
			if (np->fin == 0)
				j_front_needed = false;
			if (np->five_val == D)
			{
				//assign all inputs to ZERO
				for (int i=0; i < np->fin; i++)
				{
					np_temp = np->unodes[i];
					if (np_temp->five_val == X)
					{
						np_temp->five_val = ZERO;
						J_frontier.push_back(np_temp);
					}
					j_front_needed = false;
				}
			}
			//check if inside D frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			for(int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val != X)
					is_def = true;
			}
			if (np->fout == 0 && is_def)
				j_front_needed = false;
			for (int k = 0; k < np->fin; k++)
			{
				if ((np->five_val == D) && ((np->unodes[k]->five_val == D_bar) || (np->unodes[k]->five_val == ZERO)))
					j_front_needed = false;
			}
			if (j_front_needed && np->five_val != X)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to J-Fronter in DEFINE" << std::endl;
				J_frontier.push_back(np);
			}
			break;

		case NOT: 
			if (np->unodes[0]->five_val == X)
			{
				np->unodes[0]->five_val = not_five_val_back(np->five_val);
			}
			else if ((np->unodes[0]->five_val != X) && !five_val_conflict(np->unodes[0]->five_val, not_five_val(np->five_val)))
			{
				if (DEBUG_LUKE) std::cout << "IMPLY AND CHECK FAILED IN BACKWARDS. CONFLICT ON NOT DETECTED. TRY NEXT FRONTIER"  << std::endl;
				return 1;
			}
			break;

		case NAND: 		
			//check if np val is D_bar,

			c_val_out = ONE;
			d_val_out = D;
			//no j frontier required if input
			if (np->fin == 0)
				j_front_needed = false;
			if (np->five_val == D_bar)
			{
				//assign all inputs to 1
				for (int i=0; i < np->fin; i++)
				{
					np_temp = np->unodes[i];
					if (np_temp->five_val == X)
					{
						np_temp->five_val = ONE;
						J_frontier.push_back(np_temp);
					}
					j_front_needed = false;
				}
			}
			//check if inside D frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			for(int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val != X)
					is_def = true;
			}
			if (np->fout == 0 && is_def)
				j_front_needed = false;
			for (int k = 0; k < np->fin; k++)
			{
				//if ((np->five_val == D) && ((np->unodes[k]->five_val == D_bar) || (np->unodes[k]->five_val == ZERO)))
					//j_front_needed = false;
			}
			if (j_front_needed && np->five_val != X)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to J-Fronter in DEFINE" << std::endl;
				J_frontier.push_back(np);
			}
			
			break;

		case AND: 
			//check if np val is D,
			c_val_out = ZERO;
			d_val_out = D_bar;  //LOOK AT THIS
			if (np->five_val == D)
			{
				//assign all inputs to one
				for (int i=0; i < np->fin; i++)
				{
					np_temp = np->unodes[i];
					if (np_temp->five_val == X)
					{
						np_temp->five_val = ONE;
						J_frontier.push_back(np_temp);
					}
					j_front_needed = false;
				}
			}
			//no j frontier required if input
			if (np->fin == 0)
				j_front_needed = false;
			//check if inside J frontier already
			for(int j = 0; j < J_frontier.size(); j++) //for each element in J_frontier
			{
				if (np == J_frontier[j])
				{
					j_front_needed = false;
					if (DEBUG_LUKE) std::cout << "Duplicate node "<< np->num << " in J-Fronter" << std::endl;
				}
			}
			
			for(int j = 0; j < np->fin; j++)
			{
				if (np->unodes[j]->five_val != X)
					is_def = true;
			}
			if (np->fout == 0 && is_def)
				j_front_needed = false;
			if (j_front_needed && np->five_val != X)
			{
				if (DEBUG_LUKE) std::cout << "Adding node "<< np->num << " to J-Fronter in DEFINE" << std::endl;
				J_frontier.push_back(np);
			}
			
			break;

		default:
			if (DEBUG_LUKE) printf("No found gate type\n");
	}
	for (int i = 0; i < np->fin; i++)
	{
		np_temp = np->unodes[i];
		if (DEBUG_LUKE) std::cout << "Adding node " << np_temp->num << " to S" << std::endl;
		S.push_back(np_temp);
	}
	//while (S != empty)
		//pop line L value from stack
		//if (US nodes defined)
			//evaluate line L
			//recursively call imply_and_check(Line L)
	while (S.size() != 0)
	{
		if (DEBUG_LUKE) std::cout << "S is : " << std::endl;
		for(int i = 0; i < S.size(); i++)
			 if (DEBUG_LUKE) std::cout << S[i]->num <<std::endl;
		np_temp = S[S.size()-1];
		S.pop_back();
		//if ((np->five_val != c_val_out) && (np->five_val != d_val_out))  //if value is not controlling output, justify
			imply_and_check_define(np_temp);
	
	}
	//S is empty, np was just popped into line L
	//Return SUCCESS
	return 0;
}

int imply_and_check(NSTRUC* np){ //implies and checks circuit, does not select from j frontier. 
						//all node assignments are implied, and then checked for any inconsistencies. 

	
	if (DEBUG_LUKE) std::cout << "Running Imply and Check. " << std::endl;
	//S.push_back(np); //ready for recursive imply and check
	if (imply_and_check_recursive(np))
		return 1;
	if (imply_and_check_define(np))
		return 1;
	//print values of lines
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (DEBUG_LUKE) std::cout << "Node " << np->num << " has value " << fval(np->five_val) << std::endl;
	}
	return 0;
	
	//todo: overloading operators for enumerations
}


int dalg_run(int node_num)
{
	if (DEBUG_LUKE) std::cout << "Running Dalg_Run on node " << node_num <<std::endl;
	/*
	for (int i = 0; i < 500000000; i++)  //debug delay 
	{
		
	}
	*/
	
	
	
	
	NSTRUC* np;
	//set all node values to X
	
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (np->num == node_num)
			break;
	}
	
	//if(imply_and_check() == FAILURE)
		//return FAILURE
	if (imply_and_check(np))
		return 1;
	if (DEBUG_LUKE) std::cout << "Successfully ran imply and check \nPrinting D-Frontier" <<std::endl;
	
	for (int i = 0; i < D_frontier.size(); i++)
		if (DEBUG_LUKE) std::cout << D_frontier[i]->num << std::endl;
	
	if (DEBUG_LUKE) std::cout <<"Printing J-Frontier" << std::endl;
	for (int i = 0; i < J_frontier.size(); i++)
		if (DEBUG_LUKE) std::cout << J_frontier[i]->num << std::endl;
	//if (error not at PO)
	bool err_PO = false;
	
	for(int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (np->fout == 0 && (np->five_val == D || np->five_val == D_bar))
			err_PO = true;
	}
	e_val c_val;
	e_val not_c_val;
	e_val dc_val_in;
	e_val dc_val_out;
	e_val not_dc_val_in;
	e_val not_dc_val_out;
	vector <NSTRUC*> undef_input;  //for use with XOR/XNOR
	e_val five_val_temp = X;
	if (!err_PO)
	{
		//if D-Frontier = empty
		if (D_frontier.size() == 0)
			return 1; //return FAILURE
		
		//while (D-Frontier != empty)  //try all D-frontier gates\n
		while(D_frontier.size() != 0)
		{
			//select untried gate G from D-frontier
			np = D_frontier[D_frontier.size()-1];
			if (DEBUG_LUKE) std::cout << "Selected node " << np->num << " from D-Frontier " <<std::endl;
			D_frontier.pop_back();
			switch (np->type) //c_val = controling value of G 
			//assign cbar to every input of G with value x  //for forward implication
			{
				case BRCH: 
					if (DEBUG_LUKE) std::cout << "ERR FATAL: Unexpected Branch in D-Frontier. " <<std::endl;
					np->five_val = np->unodes[0]->five_val;
					//return 1;
					break;

				case XNOR: 
					if (DEBUG_LUKE) std::cout << "XNOR gate is in D-Frontier" << std::endl;
					for (int i = 0; i < np->fin; i++)
					{
						//if input is X, push to undef_input
						if (np->unodes[i]->five_val == X)
							undef_input.push_back(np->unodes[i]); //add to list of undefined inputs
						else 
						{
							if (five_val_temp == X)
								five_val_temp = np->unodes[i]->five_val;
							else 
								five_val_temp = not_five_val(xor_five_val(five_val_temp, np->unodes[i]->five_val));
							if (DEBUG_LUKE) std::cout << "Five val temp is " << fval(five_val_temp) << std::endl;
						}
					}
					//loop though undefined inputs and assign zero, then calculate new five_val output
					for (int i = 0; i < undef_input.size(); i++)
					{
						if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << undef_input[i]->num << std::endl;
						undef_input[i]->five_val = ZERO;
						if (DEBUG_LUKE) std::cout << "Fiveval was " << fval(np->five_val);
						five_val_temp = not_five_val(xor_five_val(five_val_temp, undef_input[i]->five_val));
						np->five_val = five_val_temp;
						if (DEBUG_LUKE) std::cout << " now its " << fval(np->five_val) << std::endl;
						J_frontier.push_back(undef_input[i]);
					}
					break;

				case XOR: 
					//loop through inputs
					if (DEBUG_LUKE) std::cout << "XOR gate is in D-Frontier" << std::endl;
					for (int i = 0; i < np->fin; i++)
					{
						//if input is X, push to undef_input
						if (np->unodes[i]->five_val == X)
							undef_input.push_back(np->unodes[i]); //add to list of undefined inputs
						else 
						{
							if (five_val_temp == X)
								five_val_temp = np->unodes[i]->five_val;
							else 
								five_val_temp = xor_five_val(five_val_temp, np->unodes[i]->five_val);
							if (DEBUG_LUKE) std::cout << "Five val temp is " << fval(five_val_temp) << std::endl;
						}
					}
					//loop though undefined inputs and assign zero, then calculate new five_val output
					for (int i = 0; i < undef_input.size(); i++)
					{
						if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << undef_input[i]->num << std::endl;
						undef_input[i]->five_val = ZERO;
						if (DEBUG_LUKE) std::cout << "Fiveval was " << fval(np->five_val);
						five_val_temp = xor_five_val(five_val_temp, undef_input[i]->five_val);
						np->five_val = five_val_temp;
						if (DEBUG_LUKE) std::cout << " now its " << fval(np->five_val) << std::endl;
						J_frontier.push_back(undef_input[i]);
					}

					break;

				case OR: 
					if (DEBUG_LUKE) std::cout << "OR gate is in D-Frontier" << std::endl;
					c_val = ONE;
					not_c_val = ZERO;
					dc_val_in = D;
					dc_val_out = D;
					not_dc_val_in = D_bar;
					not_dc_val_out = D_bar;
					break;

				case NOR: 
					if (DEBUG_LUKE) std::cout << "NOR gate is in D-Frontier" << std::endl;
					c_val = ONE;
					not_c_val = ZERO;
					dc_val_in = D;
					dc_val_out = D_bar;
					not_dc_val_in = D_bar;
					not_dc_val_out = D;  //MAYBE??
					break;

				case NOT: 
					if (DEBUG_LUKE) std::cout << "NOT GATE IN D FRONTIER. ILLEGAL!!!" << std::endl;
					np->five_val = not_five_val(np->unodes[0]->five_val);
					break;

				case NAND: 
					if (DEBUG_LUKE) std::cout << "NAND gate is in D-Frontier" << std::endl;
					c_val = ZERO;
					not_c_val = ONE;
					dc_val_in = D_bar;
					dc_val_out = D;
					not_dc_val_in = D;
					not_dc_val_out = D_bar;
					break;

				case AND: 
					if (DEBUG_LUKE) std::cout << "NAND gate is in D-Frontier" << std::endl;
					c_val = ZERO;
					not_c_val = ONE;
					dc_val_in = D_bar;
					dc_val_out = D_bar;
					not_dc_val_in = D;
					not_dc_val_out = D;
					break;

				default:
					printf("No found gate type\n");
			}
			//assign cbar to every input of G with value x  //for forward implication
			if ((np->type != XOR) && (np->type != XNOR) &&  (np->type != NOT) &&  (np->type != BRCH))
			{
				for (int i = 0; i < np->fin; i++)
				{
					if (DEBUG_LUKE) std::cout << "J node " << np->unodes[i]->num << " has value " << fval(np->unodes[i]->five_val) <<std::endl;
					if (np->unodes[i]->five_val == X)
					{
							np->unodes[i]->five_val = not_c_val;
							if (np->unodes[i]->fin != 0) //if not PI, 
								J_frontier.push_back(np->unodes[i]);
					}
					else
					{
						if (np->unodes[i]->five_val == not_dc_val_in)
						{
							np->five_val = not_dc_val_out;
						}
						else if (np->unodes[i]->five_val == dc_val_in)
						{
							np->five_val = dc_val_out;
						}
					}
					
				}
			}
			if (DEBUG_LUKE) std::cout << "D frontier node " << np->num << " has value " << fval(np->five_val) <<std::endl;
			
			//if (dalg_run == SUCCESS) 
				//return SUCCESS
			if (!dalg_run(np->num))
				return 0;
		}
	}
	//if(J-Frontier = empty)
		//return SUCCESS
	if (J_frontier.size() == 0) return 0;
	e_val c_val_out;
	e_val d_val_out;
	e_val c_val_in;
	e_val d_val_in;
	e_val not_c_val_in;
	
	
	while(J_frontier.size() != 0)
	{
		int num_ones = 0;
		vector <NSTRUC*> J_undef_input;  //for use with XOR/XNOR
		five_val_temp = X;
		bool in_justified = true;
		np = J_frontier[J_frontier.size()-1];
		J_frontier.pop_back();
		if (DEBUG_LUKE) std::cout << "Selecting J-Frontier node " << np->num << std::endl;
		switch (np->type) 
		{
			case BRCH: 
				if (np->unodes[0]->five_val == X)
					np->unodes[0]->five_val = five_val_back(np->five_val);
				else 
				{
					if (!five_val_equal(np->unodes[0]->five_val, np->five_val))
						return 1;
				}
				break;

			case XNOR: 
				if (DEBUG_LUKE) std::cout << "XNOR gate is in J-Frontier" << std::endl;
				for (int i = 0; i < np->fin; i++)
				{
					//if input is X, push to J_undef_input
					if (np->unodes[i]->five_val == X)
						J_undef_input.push_back(np->unodes[i]); //add to list of undefined inputs
					else
					{
						if ((np->unodes[i]->five_val == D) || (np->unodes[i]->five_val == ONE))
						{
							if (DEBUG_LUKE) std::cout << "input " << np->unodes[i]->num << " has value " << fval(np->unodes[i]->five_val) << " incrementing num_ones" << std::endl;
							num_ones++;
						}
					}
				}
				if (DEBUG_LUKE) std::cout << "Num ones is : " << num_ones << std::endl;
				if ((np->five_val == D) || (np->five_val == ONE))
				{
					if (num_ones % 2 == 0) //assign all X values to 0 or all values to 1 **TODO
					{						
						for (int j = 0; j < 3; j++)
						{
							if (j == 0) //first try, set all inputs to zero
							{
								for (int i = 0; i < J_undef_input.size(); i++)
								{
									if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ZERO;
									//J_frontier.push_back(J_undef_input[i]);
								}
								for (int k = 0; k < np->fin; k++)
								{
									if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
									if (dalg_run(np->unodes[k]->num))
									{
										in_justified = false;
										break;
									}
								}
								if (in_justified)
									break;
							}
							else if (j == 1) //second try, set all inputs to ONE
							{
								for (int i = 0; i < J_undef_input.size(); i++)
								{
									if (DEBUG_LUKE) std::cout << "Assigning ONE to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ONE;
									//J_frontier.push_back(J_undef_input[i]);
								}
								for (int k = 0; k < np->fin; k++)
								{
									if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
									if (dalg_run(np->unodes[k]->num))
									{
										in_justified = false;
										break;
									}
								}
								if (in_justified)
									break;
							}
							else
							{
								if (DEBUG_LUKE) std::cout << "GATE JUSTIFICATION FAILED, TRY NEXT D-FRONTIER" << std::endl;
								return 1;
							}
						}
					}
					if (num_ones % 2 == 1) //assign all One X values to 1, the rest to 0
					{
						for (int j = 0; j < J_undef_input.size(); j++)
						{
							J_undef_input[j]->five_val = ONE; //assign one input to ONE and the rest to 0
							if (DEBUG_LUKE) std::cout << "Assigning ONE to input " << J_undef_input[j]->num << std::endl;
							for (int i = 0; i < J_undef_input.size(); i++)
							{
								if (J_undef_input[i] != J_undef_input[j])
								{
									if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ZERO;
								}
							}
							for (int k = 0; k < np->fin; k++)
							{
								if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
								if (dalg_run(np->unodes[k]->num))
								{
									in_justified = false;
									break;
								}
							}
							if (in_justified)
								break;
						}
					}						
				}
				else if ((np->five_val == D_bar) || (np->five_val == ZERO))
				{
					//number of high inputs needs to be even
					if (num_ones % 2 == 1) //assign all X values to 0 or all to zero TODO
					{
						for (int j = 0; j < 3; j++)
						{
							if (j == 0) //first try, set all inputs to zero
							{
								for (int i = 0; i < J_undef_input.size(); i++)
								{
									if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ZERO;
									//J_frontier.push_back(J_undef_input[i]);
								}
								for (int k = 0; k < np->fin; k++)
								{
									if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
									if (dalg_run(np->unodes[k]->num))
									{
										in_justified = false;
										break;
									}
								}
								if (in_justified)
									break;
							}
							else if (j == 1) //second try, set all inputs to ONE
							{
								for (int i = 0; i < J_undef_input.size(); i++)
								{
									if (DEBUG_LUKE) std::cout << "Assigning ONE to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ONE;
									//J_frontier.push_back(J_undef_input[i]);
								}
								for (int k = 0; k < np->fin; k++)
								{
									if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
									if (dalg_run(np->unodes[k]->num))
									{
										in_justified = false;
										break;
									}
								}
								if (in_justified)
									break;
							}
							else
							{
								if (DEBUG_LUKE) std::cout << "GATE JUSTIFICATION FAILED, TRY NEXT D-FRONTIER" << std::endl;
								return 1;
							}
						}
					}
					if (num_ones % 2 == 0) //assign all One X values to 1, the rest to 0
					{
						for (int j = 0; j < J_undef_input.size(); j++)
						{
							J_undef_input[j]->five_val = ONE; //assign one input to ONE and the rest to 0
							if (DEBUG_LUKE) std::cout << "Assigning ONE to input " << J_undef_input[j]->num << std::endl;
							for (int i = 0; i < J_undef_input.size(); i++)
							{
								if (J_undef_input[i] != J_undef_input[j])
								{
									if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ZERO;
								}
							}
							for (int k = 0; k < np->fin; k++)
							{
								if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
								if (dalg_run(np->unodes[k]->num))
								{
									in_justified = false;
									break;
								}
							}
							if (in_justified)
								break;
						}
					}
					
				}
				else 
					if (DEBUG_LUKE) std::cout << "Why is X in J frontier??" << std::endl;
				
				break;

			case XOR:
				if (DEBUG_LUKE) std::cout << "XOR gate is in J-Frontier" << std::endl;
				for (int i = 0; i < np->fin; i++)
				{
					//if input is X, push to J_undef_input
					if (np->unodes[i]->five_val == X)
						J_undef_input.push_back(np->unodes[i]); //add to list of undefined inputs
					else
					{
						if ((np->unodes[i]->five_val == D) || (np->unodes[i]->five_val == ONE))
						{
							if (DEBUG_LUKE) std::cout << "input " << np->unodes[i]->num << " has value " << fval(np->unodes[i]->five_val) << " incrementing num_ones" << std::endl;
							num_ones++;
						}
					}
				}
				if (DEBUG_LUKE) std::cout << "Num ones is : " << num_ones << std::endl;
				if ((np->five_val == D) || (np->five_val == ONE))
				{
					if (num_ones % 2 == 1) //assign all X values to 0 or all values to 1 **TODO
					{
						for (int i = 0; i < J_undef_input.size(); i++)
						{
							if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
							J_undef_input[i]->five_val = ZERO;
							//J_frontier.push_back(J_undef_input[i]);
						}
					}
					if (num_ones % 2 == 0) //assign all One X values to 1, the rest to 0
					{
						for (int j = 0; j < J_undef_input.size(); j++)
						{
							J_undef_input[j]->five_val = ONE; //assign one input to ONE and the rest to 0
							if (DEBUG_LUKE) std::cout << "Assigning ONE to input " << J_undef_input[j]->num << std::endl;
							for (int i = 0; i < J_undef_input.size(); i++)
							{
								if (J_undef_input[i] != J_undef_input[j])
								{
									if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
									J_undef_input[i]->five_val = ZERO;
								}
							}
							for (int k = 0; k < np->fin; k++)
							{
								if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
								if (dalg_run(np->unodes[k]->num))
								{
									in_justified = false;
									break;
								}
							}
							if (in_justified)
								break;
						}
					}						
				}
				else if ((np->five_val == D_bar) || (np->five_val == ZERO))
				{
					//number of high inputs needs to be even
					if (num_ones % 2 == 0) //assign all X values to 0
					{
						for (int i = 0; i < J_undef_input.size(); i++)
						{
							if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
							J_undef_input[i]->five_val = ZERO;
							//J_frontier.push_back(J_undef_input[i]);
						}
					}
					if (num_ones % 2 == 1) //assign all One X values to 1, the rest to 0
					{
						J_undef_input[0]->five_val = ONE;
						if (DEBUG_LUKE) std::cout << "Assigning ONE to input " << J_undef_input[0]->num << std::endl;
						//J_frontier.push_back(J_undef_input[0]);
						for (int i = 1; i < J_undef_input.size(); i++)
						{
							if (DEBUG_LUKE) std::cout << "Assigning ZERO to input " << J_undef_input[i]->num << std::endl;
							J_undef_input[i]->five_val = ZERO;
							//J_frontier.push_back(J_undef_input[i]);
						}
					}
					//justify all NC outputs
					for (int k = 0; k < np->fin; k++)
					{
						if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
						if (dalg_run(np->unodes[k]->num))
							return 1;
					}
					
				}
				else 
					std::cout << "Why is X in J frontier??" << std::endl;
				
				break;

			case OR: 
				if (DEBUG_LUKE) std::cout << "OR gate is in J-Frontier" << std::endl;
				c_val_out = ONE; //output when input has c_val
				d_val_out = D;
				c_val_in = ONE;
				d_val_in = D;
				not_c_val_in = ZERO;
				break;

			case NOR: 
				if (DEBUG_LUKE) std::cout << "NOR gate is in J-Frontier" << std::endl;
				c_val_out = ZERO; //output when input has c_val
				d_val_out = D_bar;
				c_val_in = ONE;
				d_val_in = D;
				not_c_val_in = ZERO;
				break;

			case NOT: 
				if (DEBUG_LUKE) std::cout << "NOT GATE IN J FRONTIER!" << std::endl;
				if (!((np->five_val == D) || (np->five_val == D_bar)) && ((np->unodes[0]->five_val == ONE) || (np->unodes[0]->five_val == ZERO) || (np->unodes[0]->five_val == X)))
					np->unodes[0]->five_val = not_five_val_back(np->five_val);
				else
					np->unodes[0]->five_val = not_five_val(np->five_val);
				if (dalg_run(np->unodes[0]->num))
						return 1;
				break;

			case NAND: 
				if (DEBUG_LUKE) std::cout << "NAND gate is in J-Frontier" << std::endl;
				c_val_out = ONE; //output when input has c_val
				d_val_out = D;
				c_val_in = ZERO;
				d_val_in = D_bar;
				not_c_val_in = ONE;
				break;

			case AND: 
				if (DEBUG_LUKE) std::cout << "AND gate is in J-Frontier" << std::endl;
				c_val_out = ZERO; //output when input has c_val
				d_val_out = D_bar;
				c_val_in = ZERO;
				d_val_in = D_bar;
				not_c_val_in = ONE;
				break;

			default:
				if (DEBUG_LUKE) printf("No found gate type\n");
		}
		if ((np->type != XOR) && (np->type != XNOR) && (np->type != NOT) && (np->type != BRCH))
		{
			if ((np->five_val == c_val_out) || (np->five_val == d_val_out)) //if output is controlling value output
			{
				//only one value needs to be controlling value
				bool has_cont_in = false;  //controlling input
				for (int k = 0; k < np->fin; k++)
				{
					if ((np->unodes[k]->five_val == c_val_in) || (np->unodes[k]->five_val == d_val_in))
					{
						has_cont_in = true;
					}
				}
				//only one input to gate needs to be justified
				if (!has_cont_in)
				{
					for (int k = 0; k < np->fin; k++)
					{
						if (np->unodes[k]->five_val == X)
						{
							np->unodes[k]->five_val = c_val_in;
							if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
							if (!dalg_run(np->unodes[k]->num))
							{
								return 0;
							}
							else 
							{
								np->unodes[k]->five_val = not_c_val_in;
								if (DEBUG_LUKE) std::cout << "Setting node " << np->unodes[k]->num << " to value " << fval(not_c_val_in) <<std::endl;	
							}
						}
					}
				}
				
			}
			else //all values must be non controlling
			{
				if (DEBUG_LUKE) std::cout << "J node has non controlling value. All input nodes must have non-controlling values" <<std::endl;
				for (int k = 0; k < np->fin; k++)
				{
					np->unodes[k]->five_val = not_c_val_in;
					if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
					if (dalg_run(np->unodes[k]->num))
					{
						return 1;		
					}						
				}
			}
			//if all inputs are non_controlling
			
			/*
			//if current val is is controlling output
			if (np->five_val == c_val_out || np->five_val == d_val_out)
			{
				//if input does not have controlling output already
				bool has_cont_in = false;  //controlling input
				for (int k = 0; k < np->fin; k++)
				{
					if ((np->unodes[k]->five_val == c_val_in) || (np->unodes[k]->five_val == d_val_in))
					{
						has_cont_in = true;
					}
				}
				//only one input to gate needs to be justified
				if (!has_cont_in)
				{
					if (DEBUG_LUKE) std::cout << "Node does not have controlling input. Only one input needs justification" <<std::endl;
					for (int k = 0; k < np->fin; k++)
					{
						if (np->unodes[k]->five_val == X)
						{
							np->unodes[k]->five_val = c_val_in;
							if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
							if (dalg_run(np->unodes[k]->num))
								return 1;
							else 
								np->unodes[k]->five_val = not_c_val_in;
								
						}
					}
				}
				
			}*/
		}
		else if (np->type != BRCH)
		{
			//if current val is is non-controlling output
			//all inputs get NC output
			for (int k = 0; k < np->fin; k++)
			{
				if (np->unodes[k]->five_val == X)
					np->unodes[k]->five_val = not_c_val_in;
			}
			//justify all NC outputs
			for (int k = 0; k < np->fin; k++)
			{
				if (DEBUG_LUKE) std::cout << "J node US line " << np->unodes[k]->num << " has value " << fval(np->unodes[k]->five_val) <<std::endl;
				if (dalg_run(np->unodes[k]->num))
					return 1;
			}
		}
		else 
		{
			if (dalg_run(np->unodes[0]->num))
				return 1;
		}
	}
	return 0;
			
}

int dalg(char* cp)
{
	if (DEBUG_LUKE) std::cout << "Running D-Algorithm on circuit." << std::endl;
	std::string dalg_node_string;
    std::string dalg_fault_string;
	int dalg_node;
	int dalg_fault;

    int scan_val;
    NSTRUC *np;
	
    //process file name

    for (scan_val = 1; scan_val < 100; scan_val++)
    { //for n_total
        if (line[scan_val] == ' ')
            break;
        else
        {
            dalg_node_string.push_back(line[scan_val]);
        }
    }
    scan_val++;
    for (scan_val; scan_val < 100; scan_val++)
    {
        if (line[scan_val] == '\n')
            break;
        else
        {
            dalg_fault_string.push_back(line[scan_val]);
        }
    }
	
	stringstream dalg_node_stream(dalg_node_string);
	stringstream dalg_fault_stream(dalg_fault_string);

    dalg_node_stream >> dalg_node;
	dalg_fault_stream >> dalg_fault;
	
    if (DEBUG_LUKE) printf("node is %d \n", dalg_node);
    if (DEBUG_LUKE) printf("fault is %d \n", dalg_fault);
	
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		np->five_val = X;
	}
	//print values of lines
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (DEBUG_LUKE) std::cout << "Node " << np->num << " has value " << fval(np->five_val) << std::endl;
	}
	
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (np->num == dalg_node)
			break;
	}
	np->five_val = (dalg_fault == 1) ? D_bar : D;
	
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (DEBUG_LUKE) std::cout << "Node " << np->num << " has value " << fval(np->five_val) << std::endl;
	}
	
	if (dalg_run(dalg_node)) //failed
	{
		if (DEBUG_LUKE) std::cout << "FAILURE. CANNOT BE DONE" <<std::endl;
		return 1;
	}
	for (int i = 0; i < Nnodes; i++)
	{
		np = &Node[i];
		if (DEBUG_LUKE) std::cout << "Node " << np->num << " has value " << fval(np->five_val) << std::endl;
	}
	if (DEBUG_LUKE) std::cout <<"Printing D-Frontier" << std::endl;
	for (int i = 0; i < D_frontier.size(); i++)
		if (DEBUG_LUKE) std::cout << D_frontier[i]->num << std::endl;
	if (DEBUG_LUKE) std::cout <<"Printing J-Frontier" << std::endl;
	for (int i = 0; i < J_frontier.size(); i++)
		if (DEBUG_LUKE) std::cout << J_frontier[i]->num << std::endl;
	//need to fix to output correctly and change name based on circuit
	if (output_results)
	{
		ofstream myfile((circuit_name + "_DALG_" + dalg_node_string + "@" + dalg_fault_string + ".txt").c_str());
		if (DEBUG_LUKE) std::cout << "DALG test vector for " << dalg_node << "SS@" <<dalg_fault << " is: " <<std::endl;
		if (myfile.is_open())
		{
			for (int i = 0; i < Nnodes; i++)
			{
				np = &Node[i];
				if (np->type == IPT)
				{	if (np->five_val == D_bar  || np->five_val == ZERO)	
					{
						myfile << np->num <<",0" << "\n";
						if (DEBUG_LUKE) std::cout  << np->num <<",0" << std::endl;
					}
					else if (np->five_val == D  || np->five_val == ONE)
					{
						myfile << np->num << ",1" << "\n";
						if (DEBUG_LUKE) std::cout << np->num << ",1" << std::endl;
					}
					else
					{
						myfile << np->num <<",X" << "\n";
						if (DEBUG_LUKE) std::cout << np->num <<",X" << std::endl;
					}
				}
			}
			myfile.close();
		}
		else
			std::cout << "Unable to open file" << std::endl;
	}
	
	D_frontier.clear();
	J_frontier.clear();
	return 0;
}





//----------ATPG BEGIN---------------
vector<float> FC_ATPG;
vector<float> FC_ATPG_new;
vector<string> Faults_ATPG;
vector< vector<int> > ATPG_Vect; //test vectors (vector of vectors for ATPG)
int rtg_sat(char *cp)
{
    //dissect input arguemnts to data structures
    std::cout << "Running Random Test Generation until saturation\n";
	

    int n_total = 0; //the number of total random test patterns to generate
    int n_tfcr = 1;  //thee frequency of FC report
	NSTRUC* np;

    vector<int> pri_in; //primary input vector to be filled by referencing Nodes in base struct;
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->type == 0)
            pri_in.push_back(np->num);
    }
	if (DEBUG_LUKE) 
	{
		for (int i = 0; i < pri_in.size(); i++)
		{
			std::cout << pri_in[i] << " is primary input." << std::endl;
		}
	}

    //calculate total faults in circuit
    int tot_faults = Nnodes * 2;
    float cur_fc;
    
    //add pi to pri_in;

	srand(time(NULL));
	vector<int> current_vector;
    vector< vector<string> > fault_results;
	bool fc_satisfied = false;
    while (!fc_satisfied)
    {
		for (int j = 0; j < pri_in.size(); j++) //pri_in.size()
		{
			int rand_bit = rand() % 2;
			current_vector.push_back(rand_bit);
		}
		if (DEBUG_LUKE) 
		{
			std::cout << "Vector is : " << std::endl;
			for (int j = 0; j < current_vector.size(); j++)
				std::cout << current_vector[j] << " ";
			std::cout << std::endl;
		}
		bool isnew = true;
		for(int j = 0; j < ATPG_Vect.size(); j++)
		{
			if (current_vector == ATPG_Vect[j])
			{
				isnew = false;
			}
		}
		output_results = false;
				
		fault_results.push_back(dfs_single(pri_in, current_vector, cp)); 
        for (int i = 0; i < fault_results.size(); i++)
        {
            if (DEBUG_LUKE) std::cout << "Resulting Fault List or Vector " << i << " is : " << std::endl;
            for (int j = 0; j < fault_results[i].size(); j++)
                if (DEBUG_LUKE) std::cout << fault_results[i][j] << " ";
            if (DEBUG_LUKE) std::cout << std::endl;
        }
        int duplicate = 0;
        for (int i = 0; i < fault_results.size(); i++) //for each fault vector
        {
            for (int j = 0; j < fault_results[i].size(); j++) //for each fault in vector
            {
                duplicate = 0;
                for (int k = 0; k < Faults_ATPG.size(); k++) //for each prevously covered fault
                {
                    if (fault_results[i][j] == Faults_ATPG[k])
                    {
                        duplicate = 1;
                        if (DEBUG_LUKE) std::cout << "Fault " << fault_results[i][j] << "is a duplicate" << std::endl;
                    }
                }
                if (duplicate == 0)
				{
                    Faults_ATPG.push_back(fault_results[i][j]);
					if (DEBUG_LUKE) std::cout << "Adding fault " << fault_results[i][j] << " to fault list." << std::endl;
				}
            }
        }
        cur_fc = ((float)Faults_ATPG.size() / (float)tot_faults) * 100.0;
		
		if (ATPG_Vect.size() > 1 && DEBUG_LUKE)
			std::cout << "difference between last FC " << FC_ATPG[FC_ATPG.size() - 1] << " and current FC " << FC_ATPG[FC_ATPG.size()-2] << " is " << FC_ATPG[FC_ATPG.size() - 1] - FC_ATPG[FC_ATPG.size()-2] << std::endl;
		
		if (ATPG_Vect.size() < 2) //fill up first few vectors
		{
			ATPG_Vect.push_back(current_vector);
			FC_ATPG_new.push_back(cur_fc);
		}
		else if (isnew && (cur_fc > FC_ATPG[FC_ATPG.size()-1]) )  // if Fault coverage improves, and it is a new vector we add to list of vectors. 
		{
			if (DEBUG_LUKE) std::cout << "Adding vector" << std::endl;
			FC_ATPG_new.push_back(cur_fc);
			ATPG_Vect.push_back(current_vector);
		}
		if (DEBUG_LUKE) std::cout << "pushing FC_ATPG" << std::endl;
		FC_ATPG.push_back(cur_fc); //for calculating saturation
			
		
		if (DEBUG_LUKE) 
		{
			std::cout << "Faults detected is: " << std::endl;
			for (int i = 0; i < Faults_ATPG.size(); i++)
			{
				std::cout << Faults_ATPG[i] << std::endl;
			}
		}
        if (DEBUG_LUKE) std::cout << std::setprecision(2) << std::fixed;
        if (DEBUG_LUKE) std::cout << "Fault coverage at test pattern is " << cur_fc << std::endl;
		current_vector.clear();
		
		//check if saturation of FC
		int FC_hist = 5;
		float FC_thresh = 0.25;
		float curr_FC_thresh = 0.0;
		if (FC_ATPG.size() > FC_hist)
		{
			for (int i = FC_ATPG.size() - FC_hist + 1; i < FC_ATPG.size(); i++)
			{
				curr_FC_thresh += FC_ATPG[i] - FC_ATPG[i-1];
				if (DEBUG_LUKE) std::cout << "FC thresh is " << curr_FC_thresh << std::endl;
				
			}
			if ((curr_FC_thresh < FC_thresh) || FC_ATPG[FC_ATPG.size() - 1] == 100.0)
				fc_satisfied = true;
		}
    }
	std::cout << "Finished random test generation and application" << std::endl;
	if (DEBUG_LUKE) 
		for (int i = 0; i < FC_ATPG.size(); i++)
		{
			std::cout << FC_ATPG[i] << std::endl;
		}
	
}

vector < NSTRUC* > ATPG_CP; //atpg checkpoint
vector < string > ATPG_RFL;
int ATPG_rfl_func()
{
	NSTRUC* np;
	
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->type == 0 || np->type == 1)
        {
            ATPG_CP.push_back(np);
			stringstream ss;
			ss << np->num;
			string nodestr = ss.str();
			ATPG_RFL.push_back(nodestr + "@1");
			ATPG_RFL.push_back(nodestr + "@0");
        }
    }
	if (DEBUG_LUKE) 
	{
		std::cout << "Reduced fault list is: " << std::endl;
		for (int i = 0; i < ATPG_RFL.size(); i++)
			std::cout << ATPG_RFL[i] << std::endl;
	
		std::cout << "ATPG RFL function called on circuit" << std::endl;
	}
}


vector < string > ATPG_FFL;  //full fault list (not including checkpoints)
int ATPG_ffl_func()
{
	NSTRUC* np;
	
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        //if (!(np->type == 0 || np->type == 1))
        //{
			ATPG_CP.push_back(np);
			stringstream ss;
			ss << np->num;
			string nodestr = ss.str();
			ATPG_FFL.push_back(nodestr + "@1");
			ATPG_FFL.push_back(nodestr + "@0");
		//}
        
    }
	if (DEBUG_LUKE)
	{
		std::cout << "Total fault list without checkpoints is: " << std::endl;
		for (int i = 0; i < ATPG_FFL.size(); i++)
			std::cout << ATPG_FFL[i] << std::endl;
	}
	
    if (DEBUG_LUKE) std::cout << "ATPG FFL function called on circuit" << std::endl;
}





int atpg (char* cp)
{
	struct timespec start, end; 
	clock_gettime(CLOCK_MONOTONIC, &start); 
	
	std::cout << "Running ATPG \nReading circuit" << std::endl;
	cread(cp);
	char line_temp[100]; //temporary storage of variable line
	for (int i = 0; i < 100; i++)
		line_temp[i] = line[i];
	string temp_str = "temp_ignore_this.txt";
	for (int i = 0; i < temp_str.size(); i++)
		line[i] = temp_str[i];
	rtg_sat(cp);
	
	if (DEBUG_LUKE) 
	{
		std::cout << "List of vectors is: " << std::endl;
		for (int i = 0; i < ATPG_Vect.size(); i++)
		{
			//std::cout << "Vector " << i << " is : " << std::endl;
			for (int j = 0; j < ATPG_Vect[i].size(); j++)
				std::cout << ATPG_Vect[i][j] << " ";
			std::cout << std::endl;
		}
	}
	
	for (int i = 0; i < 100; i++) //line to all spaces
		line[i] = ' ';
	
	ATPG_rfl_func(); //find reduced fault list 
	
	vector < string > missing_checkpoint_fault;
	//iterate through RFL, break upon match, else push to missing fault
	for (int i = 0; i < ATPG_RFL.size(); i++)
	{
		bool fault_present = false;
		for (int j = 0; j < Faults_ATPG.size(); j++)
		{
			if (ATPG_RFL[i] == Faults_ATPG[j])
			{
				fault_present = true;
				break;
			}
		}
		if (!fault_present)
		{
			if (DEBUG_LUKE) std::cout << "Fault " << ATPG_RFL[i] << " is missing. Adding to missing list" << std::endl;
			missing_checkpoint_fault.push_back(ATPG_RFL[i]);
			
		}
	}
	string delimiter = "@";

	size_t pos = 0;
	string fnode;
	string ffault;
	NSTRUC* np;
	vector < int > curr_vect;
	vector<int> pri_in; //primary input vector to be filled by referencing Nodes in base struct;
    for (int i = 0; i < Nnodes; i++)
    {
        np = &Node[i];
        if (np->type == 0)
            pri_in.push_back(np->num);
    }
	std::cout << "Running ATPG on missing checkpoint faults" << std::endl;
	for (int i = 0; i < missing_checkpoint_fault.size(); i++)
	{
		string s = missing_checkpoint_fault[i];
		std::cout << "Checking missing fault " << s << std::endl;
		bool f_found = false;
		while ((pos = s.find(delimiter)) != std::string::npos) {
			fnode = s.substr(0, pos);
			if (DEBUG_LUKE) std::cout << fnode << std::endl;
			s.erase(0, pos + delimiter.length());
			break;
		}
		ffault = s;
		if (DEBUG_LUKE) std::cout << ffault << std::endl;
		
		line[0] = ' ';
		int lineval = 1;
		for (lineval = 1; lineval < fnode.size() + 1; lineval++)
			line[lineval] = fnode[lineval - 1];
		line[lineval] = ' ';
		lineval++;
		line[lineval] = ffault[0];
		
		if (dalg(cp))
		{
			if (DEBUG_LUKE) std::cout << "FAILED TO FIND VECTOR FOR FAULT. CONTUNUE" << std::endl;
			continue;
		}
		
		if (DEBUG_LUKE) std::cout << "New vector to be added is: " << std::endl;
		for (int j = 0; j < Nnodes; j++)
		{
			np = &Node[j];
			if (np->type == IPT)
			{	if (np->five_val == D_bar  || np->five_val == ZERO)	
				{
					curr_vect.push_back(0);
					if (DEBUG_LUKE) std::cout  << np->num <<",0" << std::endl;
				}
				else if (np->five_val == D  || np->five_val == ONE)
				{
					curr_vect.push_back(1);
					if (DEBUG_LUKE) std::cout << np->num << ",1" << std::endl;
				}
				else
				{
					int randnum = rand() % 2;
					curr_vect.push_back(randnum);
					if (DEBUG_LUKE) std::cout << np->num << "," << randnum << std::endl;
				}
			}
		}
		//check if curr_vect is in the set of vectors already
		bool isnew = true;
		for(int j = 0; j < ATPG_Vect.size(); j++)
		{
			if (curr_vect == ATPG_Vect[j])
			{
				curr_vect.clear();
				isnew = false;
			}
		}
		if (!isnew)
			continue;

		vector <string> new_faults;
		new_faults = dfs_single(pri_in, curr_vect, cp); 
		if (DEBUG_LUKE) 
		{
			std::cout << "Resulting Fault List or Vector " << i << " is : " << std::endl;
			for (int j = 0; j < new_faults.size(); j++)
			{
				std::cout << new_faults[j]<< " ";
				
			}
			std::cout << std::endl;
		}
		
        int duplicate = 0;
		float cur_fc;
        for (int j = 0; j < new_faults.size(); j++) //for each fault vector
        {
			duplicate = 0;
			for (int k = 0; k < Faults_ATPG.size(); k++) //for each prevously covered fault
			{
				if (new_faults[j] == Faults_ATPG[k])
				{
					duplicate = 1;
					if (DEBUG_LUKE) std::cout << "Fault " << new_faults[j] << "is a duplicate" << std::endl;
				}
			}
			if (duplicate == 0)
			{
				Faults_ATPG.push_back(new_faults[j]);
				if (DEBUG_LUKE) std::cout << "Fault " << new_faults[j] << "is a new fault. Adding to fault list" << std::endl;

			}
        }
		int tot_faults = Nnodes * 2;
        cur_fc = ((float)Faults_ATPG.size() / (float)tot_faults) * 100.0;
		FC_ATPG.push_back(cur_fc);
		if (cur_fc > FC_ATPG_new[FC_ATPG_new.size()-1])
		{
			FC_ATPG_new.push_back(cur_fc);
			ATPG_Vect.push_back(curr_vect);
		}
		
		curr_vect.clear();
	}
	if (DEBUG_LUKE) 
	{
		for (int i = 0; i < FC_ATPG.size(); i++)
		{
			std::cout << FC_ATPG[i] << std::endl;
		}
		
		std::cout << "List of vectors is: " << std::endl;
		for (int i = 0; i < ATPG_Vect.size(); i++)
		{
			//std::cout << "Vector " << i << " is : " << std::endl;
			for (int j = 0; j < ATPG_Vect[i].size(); j++)
				std::cout << ATPG_Vect[i][j] << " ";
			std::cout << std::endl;
		}
	}
	float FC_before_non_CP = FC_ATPG_new[FC_ATPG_new.size() - 1];
	/*
	for (int i = 0; i < FC_ATPG_new.size(); i++)
    {
        std::cout << FC_ATPG_new[i] << std::endl;
    }
	std::cout << "Number of faults detected is " << Faults_ATPG.size() << " and total faults in circuit is " << Nnodes * 2 << std::endl;
	std::cout << "Number of faults detected is " << Faults_ATPG.size() << " and total faults in circuit is " << Nnodes * 2 << std::endl;
	std::cout << "Faults detected is: " << std::endl;
	for (int i = 0; i < Faults_ATPG.size(); i++)
	{
		std::cout << Faults_ATPG[i] << std::endl;
	}
	*/
	vector < string > missing_fault;
	std::cout << "Running ATPG on missing non-checkpoint faults" << std::endl;
	if (FC_ATPG_new[FC_ATPG_new.size()-1] != 100.0) //if fault coverage is not at 100%
	{
		ATPG_ffl_func();  //find total fault list
		//iterate through FFL, break upon match, else push to missing fault
		pos = 0;
		for (int i = 0; i < ATPG_FFL.size(); i++)
		{
			bool fault_present = false;
			for (int j = 0; j < Faults_ATPG.size(); j++)
			{
				if (ATPG_FFL[i] == Faults_ATPG[j])
				{
					fault_present = true;
					break;
				}
			}
			if (!fault_present)
			{
				if (DEBUG_LUKE) std::cout << "Fault " << ATPG_FFL[i] << " is missing. Adding to missing list" << std::endl;
				missing_fault.push_back(ATPG_FFL[i]);
				
			}
		}
		for (int i = 0; i < missing_fault.size(); i++)
		{
			string s = missing_fault[i];
			std::cout << "Checking missing fault " << s << std::endl;
			bool f_found = false;
			while ((pos = s.find(delimiter)) != std::string::npos) {
				fnode = s.substr(0, pos);
				if (DEBUG_LUKE) std::cout << fnode << std::endl;
				s.erase(0, pos + delimiter.length());
				break;
			}
			ffault = s;
			if (DEBUG_LUKE) std::cout << ffault << std::endl;
			
			line[0] = ' ';
			int lineval = 1;
			for (lineval = 1; lineval < fnode.size() + 1; lineval++)
				line[lineval] = fnode[lineval - 1];
			line[lineval] = ' ';
			lineval++;
			line[lineval] = ffault[0];
			
			if (dalg(cp))
				if (DEBUG_LUKE) std::cout << "FAILED TO FIND VECTOR FOR FAULT. CONTUNUE" << std::endl;
			
			if (DEBUG_LUKE) std::cout << "New vector to be added is: " << std::endl;
			for (int j = 0; j < Nnodes; j++)
			{
				np = &Node[j];
				if (np->type == IPT)
				{	if (np->five_val == D_bar  || np->five_val == ZERO)	
					{
						curr_vect.push_back(0);
						if (DEBUG_LUKE) std::cout  << np->num <<",0" << std::endl;
					}
					else if (np->five_val == D  || np->five_val == ONE)
					{
						curr_vect.push_back(1);
						if (DEBUG_LUKE) std::cout << np->num << ",1" << std::endl;
					}
					else
					{
						int randnum = rand() % 2;
						curr_vect.push_back(randnum);
						if (DEBUG_LUKE) std::cout << np->num << "," << randnum << std::endl;
					}
				}
			}
			//check if curr_vect is in the set of vectors already
			bool isnew = true;
			for(int j = 0; j < ATPG_Vect.size(); j++)
			{
				if (curr_vect == ATPG_Vect[j])
				{
					curr_vect.clear();
					isnew = false;
				}
			}
			if (!isnew)
				continue;

			vector <string> new_faults;
			new_faults = dfs_single(pri_in, curr_vect, cp); 
			if (DEBUG_LUKE) 
			{
				std::cout << "Resulting Fault List for Vector " << i << " is : " << std::endl;
				for (int j = 0; j < new_faults.size(); j++)
				{
					std::cout << new_faults[j]<< " ";
					
				}
				std::cout << std::endl;
			}
			int duplicate = 0;
			float cur_fc;
			for (int j = 0; j < new_faults.size(); j++) //for each fault vector
			{
				duplicate = 0;
				for (int k = 0; k < Faults_ATPG.size(); k++) //for each prevously covered fault
				{
					if (new_faults[j] == Faults_ATPG[k])
					{
						duplicate = 1;
						if (DEBUG_LUKE) std::cout << "Fault " << new_faults[j] << "is a duplicate" << std::endl;
					}
				}
				if (duplicate == 0)
				{
					Faults_ATPG.push_back(new_faults[j]);
					if (DEBUG_LUKE) std::cout << "Fault " << new_faults[j] << "is a new fault. Adding to fault list" << std::endl;
				}
			}
			int tot_faults = Nnodes * 2;
			cur_fc = ((float)Faults_ATPG.size() / (float)tot_faults) * 100.0;
			std::cout << "Faults detected " << Faults_ATPG.size() <<std::endl;
			std::cout << "total faults " << tot_faults <<std::endl;
			
	
			FC_ATPG.push_back(cur_fc);
			if (cur_fc > FC_ATPG_new[FC_ATPG_new.size()-1])
			{
				FC_ATPG_new.push_back(cur_fc);
				ATPG_Vect.push_back(curr_vect);
			}
			curr_vect.clear();
		}
	}
	if (DEBUG_LUKE) 
	{
		for (int i = 0; i < FC_ATPG_new.size(); i++)
		{
			std::cout << FC_ATPG_new[i] << std::endl;
		}
		std::cout << "FC before running on non-checkpoints is " << FC_before_non_CP << std::endl;
	}
	std::cout << "Fault Coverage is: " << FC_ATPG_new[FC_ATPG_new.size() - 1] << std::endl;
	int tot_faults = Nnodes * 2;
	float cur_fc = ((float)Faults_ATPG.size() / (float)tot_faults) * 100.0;
	std::cout << "Faults detected " << Faults_ATPG.size() <<std::endl;
	std::cout << "total faults " << tot_faults <<std::endl;
	std::cout << "FFL faults " << ATPG_FFL.size() <<std::endl;
	std::cout << "FC faults " << cur_fc <<std::endl;
	for (int i = 0; i < 100; i++) //replace line with circuit name;
			line[i] = line_temp[i];
	
	//debug print statement
	missing_fault.clear();
	for (int i = 0; i < ATPG_FFL.size(); i++)
	{
		bool fault_present = false;
		for (int j = 0; j < Faults_ATPG.size(); j++)
		{
			if (ATPG_FFL[i] == Faults_ATPG[j])
			{
				fault_present = true;
				break;
			}
		}
		if (!fault_present)
		{
			std::cout << "Fault " << ATPG_FFL[i] << " is missing. Adding to missing list" << std::endl;
			missing_fault.push_back(ATPG_FFL[i]);
			
		}
	}
	
	
	
	
	std::cout << "Finished ATPG" << std::endl;
	clock_gettime(CLOCK_MONOTONIC, &end); 
	double time_taken; 
    time_taken = (end.tv_sec - start.tv_sec) * 1e9; 
    time_taken = (time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9; 
	std::cout << "Time taken by program is : " << fixed 
         << time_taken << setprecision(9); 
    std::cout << " sec" << std::endl; 
	
	string report_out = circuit_name + "_" + "DALG_ATPG_"+ "report.txt";
	string patterns_out = circuit_name + "_" + "DALG_ATPG_"+ "patterns.txt";
	
	//output report
	ofstream myfile(report_out.c_str());
    if (myfile.is_open())
    {
		myfile << "Algorithm: DALG\n";
		myfile << "Circuit: " << circuit_name << "\n";
		myfile << "Fault Coverage: " << std::fixed << std::setprecision(2) << FC_ATPG_new[FC_ATPG_new.size()-1] << "%\n";
		myfile << "Time: " << std::fixed << std::setprecision(0) << time_taken << " seconds";

        myfile.close();
    }
    else
        std::cout << "Unable to open file" << std::endl;
	
	ofstream myfile2(patterns_out.c_str());
    if (myfile2.is_open())
    {
		for (int i = 0; i < pri_in.size(); i++)
		{
			myfile2 << pri_in[i];
			if (i < (pri_in.size() - 1))
				myfile2 << ",";
		}
		myfile2 << std::endl;
		for (int i = 0; i < ATPG_Vect.size(); i++)
		{
			for (int j = 0; j < ATPG_Vect[i].size(); j++)
			{
				myfile2 << ATPG_Vect[i][j];
				if (j < (ATPG_Vect[i].size() - 1))
					myfile2 << ",";
			}
			myfile2 << std::endl;
		}
		

        myfile2.close();
    }
    else
        std::cout << "Unable to open file" << std::endl;
	
	//CLEANUP
	clear();
	FC_ATPG.clear();
	Faults_ATPG.clear();
	ATPG_RFL.clear();
	ATPG_FFL.clear();
	ATPG_CP.clear();
	ATPG_Vect.clear(); //test vectors (vector of vectors for ATPG)
}




struct cmdstruc command[NUMFUNCS] = {
    {"READ", cread, EXEC},
    {"PC", pc, CKTLD},
    {"HELP", help, EXEC},
    {"QUIT", quit, EXEC},
    {"LEV", lev, CKTLD},
    {"LOGICSIM", logicsim, CKTLD},
    {"RFL", rfl, CKTLD},
    {"DFS", dfs, CKTLD},
    {"RTG", rtg, CKTLD},
    {"PFS", pfs, CKTLD},
	{"DALG", dalg, CKTLD},
    {"PODEM",podem,CKTLD},
	{"ATPG" , atpg, EXEC},
	{"ATPG_DET",atpg_det,EXEC},
	};
//------------------------------------------------------------------------------------
int main()
{
    enum e_com com;
    char cline[MAXLINE], wstr[MAXLINE], *cp;

    while (!Done)
    {
        printf("\nCommand>");
        fgets(cline, MAXLINE, stdin);
        if (sscanf(cline, "%s", wstr) != 1)
            continue;
        cp = wstr;
        while (*cp)
        {
            *cp = Upcase(*cp);
            cp++;
        }
        cp = cline + strlen(wstr);
        com = READ;
        int i;
        for (i = 0; i < 100; i++)
            line[i] = '\0';
        for (i = 0; i < 100; i++)
        {
            if (cp[i] != '\0')
                line[i] = cp[i];
            else
                break;
        }

        while (com < NUMFUNCS && strcmp(wstr, command[com].name))
        {
            //com++
            int temp = static_cast<int>(com);
            temp++;
            com = static_cast<e_com>(temp);
        }
        if (com < NUMFUNCS)
        {
            if (command[com].state <= Gstate)
                (*command[com].fptr)(cp);
            else
                printf("Execution out of sequence!\n");
        }
        else
            system(cline);
    }
}

/*========================= End of program ============================*/




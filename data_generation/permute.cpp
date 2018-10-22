// To generate obstacles permutation to generate new environments
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
using namespace std;
void combinationUtil(int arr[], int data[], int start, int end, int index, int r, int &count, int (&node)[77520][7]);

// The main function that prints all combinations of size r
// in arr[] of size n. This function mainly uses combinationUtil()


void printCombination(int arr[], int n, int r)
{
    // A temporary array to store all combination one by one





    int data[r];
 	 int count=0;
	 int node[77520][7];
	 //node[0]=0;
	 //node[1]=0;	
    // Print all combination using temprary array 'data[]'
    combinationUtil(arr, data, 0, n-1, 0, r, count, node);
	cout<<"count: "<<count<<endl;
	for (int i=0;i<10;i++){
		for (int j=0; j<7;j++)
			cout<<node[i][j]<<' ';
		cout<<endl;
	}

	


	random_shuffle(&node[0], &node[77520]);
	cout<<"count: "<<count<<endl;
	for (int i=0;i<10;i++){
		for (int j=0; j<7;j++)
			cout<<node[i][j]<<' ';
		cout<<endl;
	}


	ofstream out("obs_perm2.dat", ios::out | ios::binary);
          if(!out) {
                        cout << "Cannot open file.";
                return;
                }

          out.write((char *) &node, sizeof node);
          out.close();

	

}
 
/* arr[]  ---> Input Array
   data[] ---> Temporary array to store current combination
   start & end ---> Staring and Ending indexes in arr[]
   index  ---> Current index in data[]
   r ---> Size of a combination to be printed */
void combinationUtil(int arr[], int data[], int start, int end,
                     int index, int r, int &count, int (&node)[77520][7])
{
    // Current combination is ready to be printed, print it
    if (index == r)
    {	  int j=0;	
        for (j=0; j<r; j++){
            cout<<data[j];
				node[count][j]=data[j];}
        cout<<endl;

		  count=count+1;
        return;
    }
 
    // replace index with all possible elements. The condition
    // "end-i+1 >= r-index" makes sure that including one element
    // at index will make a combination with remaining elements
    // at remaining positions
	 int i=0;
    for (i=start; i<=end && end-i+1 >= r-index; i++)
    {
        data[index] = arr[i];
        combinationUtil(arr, data, i+1, end, index+1, r, count, node);
		  
    }
	
}



int main()
{
    int arr[] = {0,1, 2, 3, 4, 5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
    int r = 7;
    int n = sizeof(arr)/sizeof(arr[0]);
    printCombination(arr, n, r);
}



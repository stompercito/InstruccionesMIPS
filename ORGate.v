/******************************************************************
* Description
*		This is an OR gate:
* Version:
*	1.0
* Author:
*Luis David Gallegos Godoy
* email:
*	is709571@iteso.mx
* Date:
*	02/07/2019
******************************************************************/
module ORGate
(
	input A,
	input B,
	output reg C
);

always@(*)
	C = A | B;

endmodule
//andgate//

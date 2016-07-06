#include "kinematics.h"
#include "rasp_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

void reset_position ()
{
	int i;
	struct uart_data dados = {"C", 1};
	int init_ang[] = {90, 90, 180};

	uart_send (&dados);

	for (i = 0; i < 3; i++)
		int_cur_angle[i] = init_ang[i];
	
	update_cur_angle ();
}

void move_arm (float end_pos[3], FILE* logs[3])
{
	const int max_steps = 500;
	const float tol = 1;
	const float step_size = 1e-1;
	int counter = 0;

	float t[3];
	int it[3];
	float norm;

	struct uart_data dados;
	dados.data = (char*) malloc (10 * sizeof(char));
	dados.length = 10;

	float_assign (t, cur_angle);
	while (((norm = inverse_kinematics (t, end_pos, step_size, arm_len, cur_angle)) > tol) && (counter++ < max_steps))
	{
		float_assign (cur_angle, t);
		update_int_angle ();
		int_assign (it, int_cur_angle);
		sprintf (dados.data, "A%03d%03d%03d", it[0], it[1], it[2]);
		uart_send (&dados);

		usleep (300e3);

		float new_position[3], new_pos_xyz[3];
		end_position (new_position, arm_len, cur_angle);
		cyl2cart (new_pos_xyz, new_position);

		print_vec ("", new_pos_xyz, logs[0]);
		print_vec ("", t, logs[1]);
		fprintf (logs[2], "%.4f\n", norm);
	}
	update_cur_angle ();
	free (dados.data);
	fprintf (logs[2], "%.4f\n", norm);
}

int main (int argc, char* argv[])
{
	FILE* fp_log[3];
	fp_log[0] = fopen("./logs/trajectory", "w");
	fp_log[1] = fopen("./logs/angles", "w");
	fp_log[2] = fopen("./logs/norms", "w");

	if ((errno = uart_init ((argc < 2) ? "/dev/null" : argv[1])) < 0) {
		fprintf (stderr, "uart_init: initialization error.\n");
		exit(1);
	}

	int i;
	char line[128];
	float end_position[3];

	reset_position ();

	while (fgets (line, 128, stdin) != NULL)
	{
		switch (line[0])
		{
			case 'A':
			sscanf (line + 1, "%f %f %f", &end_position[0], &end_position[1], &end_position[2]);
			move_arm (end_position, fp_log);
			break;
			case 'B':
			reset_position ();
			break;
			default:
			;
		}
	}

	for (i = 0; i < 3; i++)
		fclose (fp_log[i]);

	uart_end ();

	return 0;
}

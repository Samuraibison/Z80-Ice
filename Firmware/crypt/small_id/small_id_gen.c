/*
 * Command line utility for generating a small_id_table.h header
 *
 * Copyright Debug Innovations Ltd. 2023
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include "filename.h"


// Utility version
#define SMALL_ID_GEN_VER   "V1.0"


// Constructs a user friendly time string
static char *tstr(time_t t)
{
    static char  tbuff[50];
    struct tm    local;

    localtime_s(&local, &t);
    strftime(tbuff, sizeof(tbuff), "%a %#d %b %Y %H:%M:%S", &local);

    return tbuff;
}


static int generate_file(char *cmd_line, char *out_filename, int num_table_entries, unsigned int rng_seed)
{
    time_t    time_now;
    FILE      *outfile;
    errno_t   err;
    uint8_t   r8;
    uint16_t  r16;
    int       i;

    err = fopen_s(&outfile, out_filename, "wt");

    if (err == 0)
    {
        // Get current date and time
        time(&time_now);

        if (rng_seed == 0)
        {
            // Seed not specified - generate a random seed
            rng_seed = (unsigned int)time_now;

            // Initial seed
            srand(rng_seed);

            // rand() returns a value 0-32767 but we need a
            // range of 0-65535 so XOR with a shifted value
            r16 = (uint16_t)rand() ^ ((uint16_t)rand() << 1);
            rng_seed = (unsigned int)r16;
            rng_seed <<= 16;

            // Now use a 2nd 16-bit value to get an unsigned int rng_seed
            r16 = (uint16_t)rand() ^ ((uint16_t)rand() << 1);
            rng_seed |= (unsigned int)r16;
        }

        // Seed the RNG
        srand(rng_seed);

        // File header
        fprintf(outfile, "/* Automatically generated file - do not edit\n");
        fprintf(outfile, " *\n");
        fprintf(outfile, " * NOTE: The contents of this file may be secret\n");
        fprintf(outfile, " *\n");
        fprintf(outfile, " * Generated at     : %s\n", tstr(time_now));
        fprintf(outfile, " * RNG Seed         : %u\n", rng_seed);
        fprintf(outfile, " * Invocation Line  : \"%s\"\n", cmd_line);
        fprintf(outfile, " * small_id_gen ver : %s\n", SMALL_ID_GEN_VER);
        fprintf(outfile, " */\n\n");

        fprintf(outfile, "#include <stdint.h>\n\n");
        fprintf(outfile, "static const uint8_t small_id_table[] =\n");
        fprintf(outfile, "{\n");

        for (i = 0; i < num_table_entries; i++)
        {
            // Generate rows of 8 bytes
            r8 = (uint8_t)rand();

            if ((i % 8) == 0)
                // Start of a row
                fprintf(outfile, "    ");

            if (i == (num_table_entries - 1))
            {
                // Last value
                fprintf(outfile, "0x%02X\n", r8);
                fprintf(outfile, "};\n");
            }
            else if ((i % 8) == 7)
                // End of a row
                fprintf(outfile, "0x%02X,\n", r8);
            else
                fprintf(outfile, "0x%02X, ", r8);
        }

        fprintf(outfile, "\n");
        fprintf(outfile, "#define SMALL_ID_TABLE_NUM_ENTRIES  %d\n", num_table_entries);
        fprintf(outfile, "\n");

        fclose(outfile);
    }
    else
    {
        fprintf(stderr, "small_id_gen: ERROR: Couldn't write output file\n");
        return 1;
    }

    return 0;
}


static void display_help(char *signon)
{
    printf("%s", signon);
    printf("small_id_gen: Usage is \"small_id_gen [options] [output filename]\"\n");
    printf("If the output filename is not specified, it will be 'small_id_table.h'\n");
    printf("  Options:  -h     Show this help\n");
    printf("            -n<n>  Specify the number of table entries (default is 64)\n");
    printf("            -s<n>  Specify the random number seed (default is random)\n");
    printf("            -q     Quiet mode - suppresses some messages for batch files\n");
    printf("\n");
    printf("  Example:  small_id_gen -n16 my_ids.h  Generates a table of 16 bytes in my_ids.h\n");
}


int main(int argc, char **argv)
{
    char     signon[200], cmd_line[500], out_filename[200];
    char     *param_str, *out_filename_ptr;
    int      param, value, res, num_table_entries, rng_seed;
    bool     quiet_mode;

    // Construct sign-on message
    sprintf_s(signon, sizeof(signon), "\nSmall ID Generation Utility %s\n", SMALL_ID_GEN_VER);
    strcat_s(signon, sizeof(signon), "Copyright Debug Innovations Ltd. 2023\n\n");

    // Default settings
    out_filename_ptr = NULL;
    num_table_entries = 64;
    rng_seed = 0;
    quiet_mode = false;

    // Check the number of command line parameters and print usage instructions
    if (argc > 5)
    {
        display_help(signon);
        return 1;
    }

    // Build a copy of the command line
    strcpy_s(cmd_line, sizeof(cmd_line), argv[0]);
    filename_remove_path(cmd_line);
    filename_change_extension(cmd_line, NULL);

    for (param = 1; param < argc; param++)
    {
        strcat_s(cmd_line, sizeof(cmd_line), " ");
        strcat_s(cmd_line, sizeof(cmd_line), argv[param]);
    }

    // Parse the command line parameters
    for (param = 1; param < argc; param++)
    {
        if (argv[param][0] == '-')
        {
            switch (tolower(argv[param][1]))
            {
                case 'h':
                    display_help(signon);
                    return 1;

                case 'n':
                    param_str = &(argv[param][2]);
                    if (strlen(param_str) > 0)
                        sscanf_s(param_str, "%d", &value);
                    num_table_entries = value;
                    break;

                case 's':
                    param_str = &(argv[param][2]);
                    if (strlen(param_str) > 0)
                        sscanf_s(param_str, "%d", &value);
                    rng_seed = value;
                    break;

                case 'q':
                    quiet_mode = true;
                    break;

                default:
                    printf("%s", signon);
                    fprintf(stderr, "small_id_gen: ERROR: Unknown option \"%s\"\n", argv[param]);
                    return 1;
            }
        }
        else
        {
            if (out_filename_ptr == NULL)
                out_filename_ptr = argv[param];
            else
            {
                printf("%s", signon);
                fprintf(stderr, "small_id_gen: ERROR: Too many non-option parameters\n");
                return 1;
            }
        }
    }

    // Sign on
    if (!quiet_mode)
        printf("%s", signon);

    if (argc == 1)
        printf("Use \"small_id_gen -h\" for options\n\n");

    // Construct the output filename
    if (out_filename_ptr == NULL)
        strcpy_s(out_filename, sizeof(out_filename), "small_id_table.h");
    else
        strcpy_s(out_filename, sizeof(out_filename), out_filename_ptr);

    // Write file
    res = generate_file(cmd_line, out_filename, num_table_entries, (unsigned int)rng_seed);

    if (!quiet_mode && (res == 0))
        printf("small_id_gen: Generated \"%s\"\n", out_filename);

    return res;
}


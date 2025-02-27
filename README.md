# Single-Axis ADCS Testbed Project

This is the code for the single-axis ADCS testbed project for the course AE4S10 Microsat Engineering. The files are structured as follows:

- The `pico_primary/` directory houses the code that runs on the primary Pico
- The `pico_secondary/` directory houses the code that runs on the secondary Pico
- The `miscellaneous/` directory has various code snippets used for experimentation and development of individual software components before they were integrated
- The `report_plots/` directory has the recorded data, the code, and the figures used in the report.

Running this code on the Picos requires them to be flashed with the MicroPython firmware. Development using VS Code with the `MicroPico` extension for uploading files to the boards and interacting with the REPL is recommended as a development workflow.

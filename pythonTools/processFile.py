def process_file(input_file, output_file_a, output_file_b, x, y):
    with open(input_file, "r", encoding="utf-8") as infile:
        with open(output_file_a, "w", encoding="utf-8") as file_a, open(
            output_file_b, "w", encoding="utf-8"
        ) as file_b:

            capture_b = False
            lines_to_write = []

            file_b.write("M9002\n")
            file_b.write("M9010 T3\n")
            

            for line in infile:
                line = line.strip()

                if line.endswith("(move to SM horizontal home)"):
                    capture_b = not capture_b
                    if capture_b:
                        file_b.write(line + "\n")
                    else:
                        file_b.write(line + "\n\n")
                    continue

                if capture_b:
                    file_b.write(line + "\n")
                else:
                    if line.endswith("(move to SM vertical home)"):
                        lines_to_write.append(
                            f"G0 Z{x} B-42.245 C21.613 (move to SM vertical home)\n"
                        )
                    elif line.endswith("(move to AM home)"):
                        lines_to_write.append(
                            f"G0 Z{y} B47.755 C21.613 (move to AM home)\n"
                        )
                    else:
                        modified_line = line
                        # modified_line = line.replace("F900", "F950")
                        # modified_line = modified_line.replace("S5500", "S6500")
                        lines_to_write.append(modified_line + "\n")

            for line in lines_to_write[:-1]:
                file_a.write(line)

            line = "G1 Z220 B-42.245 C21.613 F20000 (move to SM vertical home)"
            file_a.write("\n" + line + "\n")
            line = "M6000"
            file_a.write(line + "\n")

            if lines_to_write:
                file_a.write(lines_to_write[-1])

            file_b.write("M9003\n")
            file_b.write("M30\n")


# 使用示例

for fileIdx in range(0, 4):
    inputFile = "C:/Users/j46868yc/OneDrive/5AxisMaker/TO_needSupport_afterPre10/" + str(fileIdx) + ".txt"
    outputFileA = (
        "C:/Users/j46868yc/OneDrive/5AxisMaker/TO_needSupport_afterPre10/" + str(fileIdx) + "_a.txt"
    )
    outputFileB = (
        "C:/Users/j46868yc/OneDrive/5AxisMaker/TO_needSupport_afterPre10/" + str(fileIdx) + "_b.txt"
    )

    process_file(inputFile, outputFileA, outputFileB, x=45, y=170)

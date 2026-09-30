"""Create a single Excel workbook from motor-validation CSV files."""

import csv
import re
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape


DATA_DIR = Path(__file__).parent / "Data"
OUTPUT = DATA_DIR / "motor_validation_combined.xlsx"


def read_csv(path):
    with path.open(newline="") as handle:
        return list(csv.reader(handle))


def rate_from_name(path):
    match = re.search(r"_(\d+\.\d+)\.csv$", path.name)
    return match.group(1) if match else ""


def combined_rows(prefix):
    rows = []
    files = sorted(DATA_DIR.glob(f"{prefix}_*.csv"))
    header = ["Ramp Rate"]
    for index, path in enumerate(files):
        data = read_csv(path)
        if index == 0:
            header += data[0]
        for row in data[1:]:
            rows.append([rate_from_name(path)] + row)
    return [header] + rows


def column_name(number):
    result = ""
    while number:
        number, remainder = divmod(number - 1, 26)
        result = chr(65 + remainder) + result
    return result


def worksheet_xml(rows):
    cells = []
    for row_number, row in enumerate(rows, 1):
        row_cells = []
        for column_number, value in enumerate(row, 1):
            ref = f"{column_name(column_number)}{row_number}"
            text = escape(str(value))
            if row_number == 1:
                row_cells.append(
                    f'<c r="{ref}" t="inlineStr"><is><t>{text}</t></is></c>'
                )
            else:
                try:
                    float(value)
                    row_cells.append(f'<c r="{ref}"><v>{escape(str(value))}</v></c>')
                except ValueError:
                    row_cells.append(
                        f'<c r="{ref}" t="inlineStr"><is><t>{text}</t></is></c>'
                    )
        cells.append(f'<row r="{row_number}">{"".join(row_cells)}</row>')
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<sheetData>' + "".join(cells) + "</sheetData></worksheet>"
    )


def write_workbook(sheets):
    content_types = [
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">',
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>',
        '<Default Extension="xml" ContentType="application/xml"/>',
        '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>',
        '<Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>',
    ]
    for index in range(1, len(sheets) + 1):
        content_types.append(
            f'<Override PartName="/xl/worksheets/sheet{index}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        )
    content_types.append("</Types>")

    workbook = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        '<sheets>'
        + "".join(
            f'<sheet name="{escape(name)}" sheetId="{index}" r:id="rId{index}"/>'
            for index, (name, _) in enumerate(sheets, 1)
        )
        + "</sheets></workbook>"
    )
    workbook_rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        + "".join(
            f'<Relationship Id="rId{index}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{index}.xml"/>'
            for index in range(1, len(sheets) + 1)
        )
        + f'<Relationship Id="rId{len(sheets)+1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
        + "</Relationships>"
    )
    root_rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
        '</Relationships>'
    )
    styles = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<fonts count="1"><font><sz val="11"/><name val="Calibri"/></font></fonts>'
        '<fills count="2"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill></fills>'
        '<borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders>'
        '<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>'
        '<cellXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellXfs>'
        '</styleSheet>'
    )

    with zipfile.ZipFile(OUTPUT, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", "".join(content_types))
        archive.writestr("_rels/.rels", root_rels)
        archive.writestr("xl/workbook.xml", workbook)
        archive.writestr("xl/_rels/workbook.xml.rels", workbook_rels)
        archive.writestr("xl/styles.xml", styles)
        for index, (_, rows) in enumerate(sheets, 1):
            archive.writestr(f"xl/worksheets/sheet{index}.xml", worksheet_xml(rows))


def main():
    sheets = [("README", [["Motor validation workbook"], ["Source folder", str(DATA_DIR)], ["Combined sheets", "Truth_All and Jerk_Accel_All"], ["Individual sheets", "One sheet per source CSV"]])]
    sheets.append(("Truth_All", combined_rows("truth")))
    sheets.append(("Jerk_Accel_All", combined_rows("jerk_accel")))
    for path in sorted(DATA_DIR.glob("*.csv")):
        name = path.stem[:31]
        sheets.append((name, read_csv(path)))
    write_workbook(sheets)
    print(OUTPUT)


if __name__ == "__main__":
    main()

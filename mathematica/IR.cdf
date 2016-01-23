(* Content-type: application/vnd.wolfram.cdf.text *)

(*** Wolfram CDF File ***)
(* http://www.wolfram.com/cdf *)

(* CreatedBy='Mathematica 10.0' *)

(*************************************************************************)
(*                                                                       *)
(*  The Mathematica License under which this file was created prohibits  *)
(*  restricting third parties in receipt of this file from republishing  *)
(*  or redistributing it by any means, including but not limited to      *)
(*  rights management or terms of use, without the express consent of    *)
(*  Wolfram Research, Inc. For additional information concerning CDF     *)
(*  licensing and redistribution see:                                    *)
(*                                                                       *)
(*        www.wolfram.com/cdf/adopting-cdf/licensing-options.html        *)
(*                                                                       *)
(*************************************************************************)

(*CacheID: 234*)
(* Internal cache information:
NotebookFileLineBreakTest
NotebookFileLineBreakTest
NotebookDataPosition[      1064,         20]
NotebookDataLength[     50543,       1464]
NotebookOptionsPosition[     48256,       1363]
NotebookOutlinePosition[     48683,       1382]
CellTagsIndexPosition[     48640,       1379]
WindowFrame->Normal*)

(* Beginning of Notebook Content *)
Notebook[{

Cell[CellGroupData[{
Cell["IR light bumper sensor", "Title"],

Cell[CellGroupData[{

Cell["Angles and lengths", "Section"],

Cell[BoxData[
 RowBox[{
  RowBox[{"\[Gamma]", "[", 
   RowBox[{"\[Beta]_", ",", "\[Alpha]i_"}], "]"}], ":=", 
  RowBox[{"\[Beta]", "+", 
   RowBox[{"2", "\[Alpha]i"}]}]}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{"lExt", "[", 
   RowBox[{"\[Beta]_", ",", "\[Alpha]i_", ",", "S_"}], "]"}], ":=", 
  RowBox[{
   FractionBox["S", "2"], "*", 
   RowBox[{"Tan", " ", "[", 
    RowBox[{
     FractionBox[
      RowBox[{"\[Pi]", "-", "\[Beta]"}], "2"], "-", "\[Alpha]i"}], 
    "]"}]}]}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{"ir\[CapitalDelta]", "[", 
   RowBox[{"\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_"}], 
   "]"}], ":=", 
  RowBox[{
   FractionBox["S", "2"], "*", 
   RowBox[{"Sin", "[", 
    RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], "]"}], "*", 
   SuperscriptBox[
    RowBox[{"Sin", "[", 
     FractionBox[
      RowBox[{"\[Gamma]", "[", 
       RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "]"}], 
    RowBox[{"-", "1"}]], "*", 
   SuperscriptBox[
    RowBox[{"Sin", "[", 
     RowBox[{
      RowBox[{"\[Gamma]", "[", 
       RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "-", 
      RowBox[{"(", 
       RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], ")"}]}], "]"}], 
    RowBox[{"-", "1"}]]}]}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"laserOuterLenght", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_"}], 
    "]"}], ":=", 
   SqrtBox[
    RowBox[{
     SuperscriptBox[
      RowBox[{"(", 
       RowBox[{"l", "+", 
        RowBox[{"lExt", "[", 
         RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S"}], "]"}]}], ")"}], 
      "2"], "+", 
     SuperscriptBox[
      RowBox[{"ir\[CapitalDelta]", "[", 
       RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S"}], 
       "]"}], "2"], "+", 
     RowBox[{"2", "*", 
      RowBox[{"ir\[CapitalDelta]", "[", 
       RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S"}], 
       "]"}], "*", 
      RowBox[{"(", 
       RowBox[{"l", "+", 
        RowBox[{"lExt", "[", 
         RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S"}], "]"}]}], ")"}], "*", 
      RowBox[{"Cos", "[", 
       FractionBox[
        RowBox[{"\[Gamma]", "[", 
         RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "]"}]}]}]]}], 
  ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{"\[Epsilon]max", "[", 
   RowBox[{
   "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_"}], 
   "]"}], ":=", 
  RowBox[{"ArcSin", "[", 
   FractionBox[
    RowBox[{
     RowBox[{"ir\[CapitalDelta]", "[", 
      RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S"}], 
      "]"}], "*", 
     RowBox[{"Sin", "[", 
      FractionBox[
       RowBox[{"\[Gamma]", "[", 
        RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "]"}]}], 
    RowBox[{"laserOuterLenght", "[", 
     RowBox[{
     "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l"}], 
     "]"}]], "]"}]}]], "Input"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{
  RowBox[{"\[Epsilon]max", "[", 
   RowBox[{
   "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l"}], "]"}],
   "//", "Simplify"}]], "Input"],

Cell[BoxData[
 RowBox[{"ArcSin", "[", 
  FractionBox[
   RowBox[{"S", " ", 
    RowBox[{"Csc", "[", 
     RowBox[{"\[Alpha]i", "-", "\[Alpha]o", "+", "\[Beta]"}], "]"}], " ", 
    RowBox[{"Sin", "[", 
     RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], "]"}]}], 
   SqrtBox[
    RowBox[{
     SuperscriptBox[
      RowBox[{"(", 
       RowBox[{
        RowBox[{"2", " ", "l"}], "+", 
        RowBox[{"S", " ", 
         RowBox[{"Cot", "[", 
          RowBox[{"\[Alpha]i", "+", 
           FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}], "2"], "+", 
     RowBox[{"2", " ", "S", " ", 
      RowBox[{"Cot", "[", 
       RowBox[{"\[Alpha]i", "+", 
        FractionBox["\[Beta]", "2"]}], "]"}], " ", 
      RowBox[{"(", 
       RowBox[{
        RowBox[{"2", " ", "l"}], "+", 
        RowBox[{"S", " ", 
         RowBox[{"Cot", "[", 
          RowBox[{"\[Alpha]i", "+", 
           FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}], " ", 
      RowBox[{"Csc", "[", 
       RowBox[{"\[Alpha]i", "-", "\[Alpha]o", "+", "\[Beta]"}], "]"}], " ", 
      RowBox[{"Sin", "[", 
       RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], "]"}]}], "+", 
     RowBox[{
      SuperscriptBox["S", "2"], " ", 
      SuperscriptBox[
       RowBox[{"Csc", "[", 
        RowBox[{"\[Alpha]i", "+", 
         FractionBox["\[Beta]", "2"]}], "]"}], "2"], " ", 
      SuperscriptBox[
       RowBox[{"Csc", "[", 
        RowBox[{"\[Alpha]i", "-", "\[Alpha]o", "+", "\[Beta]"}], "]"}], "2"], 
      " ", 
      SuperscriptBox[
       RowBox[{"Sin", "[", 
        RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], "]"}], "2"]}]}]]], 
  "]"}]], "Output"]
}, Open  ]]
}, Open  ]],

Cell[CellGroupData[{

Cell["Laser min/max lengths", "Section"],

Cell[CellGroupData[{

Cell["Line equations", "Subsection"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{
    RowBox[{"laser", "[", "\[Epsilon]_", "]"}], "[", "x_", "]"}], ":=", 
   RowBox[{
    RowBox[{"Tan", "[", "\[Epsilon]", "]"}], "*", "x"}]}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{
    RowBox[{"diodeOuter", "[", 
     RowBox[{
     "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_"}],
      "]"}], "[", "x_", "]"}], ":=", " ", 
   RowBox[{
    RowBox[{"S", "/", "2"}], "-", 
    RowBox[{
     RowBox[{"Tan", "[", 
      RowBox[{
       RowBox[{"\[Pi]", "/", "2"}], "-", 
       RowBox[{"(", 
        RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], ")"}], "-", 
       RowBox[{"(", 
        RowBox[{
         RowBox[{
          RowBox[{"(", 
           RowBox[{"\[Pi]", "-", "\[Beta]"}], ")"}], "/", "2"}], "-", 
         "\[Alpha]i"}], ")"}]}], "]"}], "*", 
     RowBox[{"(", 
      RowBox[{"x", "-", "l"}], ")"}]}]}]}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{
    RowBox[{"diodeCenter", "[", 
     RowBox[{"\[Beta]_", ",", "\[Alpha]i_", ",", "S_", ",", "l_"}], "]"}], 
    "[", "x_", "]"}], ":=", 
   RowBox[{
    RowBox[{"diodeOuter", "[", 
     RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "0", ",", "S", ",", "l"}], 
     "]"}], "[", "x", "]"}]}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{
    RowBox[{"diodeInner", "[", 
     RowBox[{"\[Beta]_", ",", "\[Alpha]i_", ",", "S_", ",", "l_"}], "]"}], 
    "[", "x_", "]"}], ":=", 
   RowBox[{
    RowBox[{"S", "/", "2"}], "-", 
    RowBox[{
     RowBox[{"Tan", "[", 
      RowBox[{
       RowBox[{"\[Pi]", "/", "2"}], "-", 
       RowBox[{"(", 
        RowBox[{
         RowBox[{
          RowBox[{"(", 
           RowBox[{"\[Pi]", "-", "\[Beta]"}], ")"}], "/", "2"}], "-", 
         "\[Alpha]i"}], ")"}]}], "]"}], "*", 
     RowBox[{"(", 
      RowBox[{"x", "-", "l"}], ")"}]}]}]}], ";"}]], "Input"]
}, Closed]],

Cell[CellGroupData[{

Cell["Intersections", "Subsection"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{"Solve", "[", 
  RowBox[{
   RowBox[{
    RowBox[{
     RowBox[{"laser", "[", "\[Epsilon]", "]"}], "[", "x", "]"}], "\[Equal]", 
    RowBox[{"-", 
     RowBox[{
      RowBox[{"diodeInner", "[", 
       RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l"}], "]"}], "[", 
      "x", "]"}]}]}], ",", "x"}], "]"}]], "Input"],

Cell[BoxData[
 RowBox[{"{", 
  RowBox[{"{", 
   RowBox[{"x", "\[Rule]", 
    FractionBox[
     RowBox[{
      RowBox[{"-", "S"}], "+", 
      RowBox[{"2", " ", "l", " ", 
       RowBox[{"Cot", "[", 
        RowBox[{"\[Alpha]i", "+", 
         RowBox[{
          FractionBox["1", "2"], " ", 
          RowBox[{"(", 
           RowBox[{
            RowBox[{"-", "\[Pi]"}], "+", "\[Beta]"}], ")"}]}]}], "]"}]}]}], 
     RowBox[{"2", " ", 
      RowBox[{"(", 
       RowBox[{
        RowBox[{"Cot", "[", 
         RowBox[{"\[Alpha]i", "+", 
          RowBox[{
           FractionBox["1", "2"], " ", 
           RowBox[{"(", 
            RowBox[{
             RowBox[{"-", "\[Pi]"}], "+", "\[Beta]"}], ")"}]}]}], "]"}], "+", 
        
        RowBox[{"Tan", "[", "\[Epsilon]", "]"}]}], ")"}]}]]}], "}"}], 
  "}"}]], "Output"]
}, Open  ]],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"xmin", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "S_", ",", "l_", ",", "\[Epsilon]_"}],
     "]"}], ":=", 
   FractionBox[
    RowBox[{
     RowBox[{"-", "S"}], "+", 
     RowBox[{"2", " ", "l", " ", 
      RowBox[{"Cot", "[", 
       RowBox[{"\[Alpha]i", "+", 
        RowBox[{
         FractionBox["1", "2"], " ", 
         RowBox[{"(", 
          RowBox[{
           RowBox[{"-", "\[Pi]"}], "+", "\[Beta]"}], ")"}]}]}], "]"}]}]}], 
    RowBox[{"2", " ", 
     RowBox[{"(", 
      RowBox[{
       RowBox[{"Cot", "[", 
        RowBox[{"\[Alpha]i", "+", 
         RowBox[{
          FractionBox["1", "2"], " ", 
          RowBox[{"(", 
           RowBox[{
            RowBox[{"-", "\[Pi]"}], "+", "\[Beta]"}], ")"}]}]}], "]"}], "+", 
       RowBox[{"Tan", "[", 
        RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}], ")"}]}]]}], 
  ";"}]], "Input"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{"Solve", "[", 
  RowBox[{
   RowBox[{
    RowBox[{
     RowBox[{"laser", "[", "\[Epsilon]", "]"}], "[", "x", "]"}], "\[Equal]", 
    RowBox[{
     RowBox[{"diodeOuter", "[", 
      RowBox[{
      "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l"}], 
      "]"}], "[", "x", "]"}]}], ",", "x"}], "]"}]], "Input"],

Cell[BoxData[
 RowBox[{"{", 
  RowBox[{"{", 
   RowBox[{"x", "\[Rule]", 
    FractionBox[
     RowBox[{"S", "+", 
      RowBox[{"2", " ", "l", " ", 
       RowBox[{"Cot", "[", 
        RowBox[{"\[Alpha]o", "+", 
         FractionBox[
          RowBox[{"\[Pi]", "-", "\[Beta]"}], "2"]}], "]"}]}]}], 
     RowBox[{"2", " ", 
      RowBox[{"(", 
       RowBox[{
        RowBox[{"Cot", "[", 
         RowBox[{"\[Alpha]o", "+", 
          FractionBox[
           RowBox[{"\[Pi]", "-", "\[Beta]"}], "2"]}], "]"}], "+", 
        RowBox[{"Tan", "[", "\[Epsilon]", "]"}]}], ")"}]}]]}], "}"}], 
  "}"}]], "Output"]
}, Open  ]],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"xmax", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]o_", ",", "S_", ",", "l_", ",", "\[Epsilon]_"}],
     "]"}], ":=", 
   FractionBox[
    RowBox[{"S", "+", 
     RowBox[{"2", " ", "l", " ", 
      RowBox[{"Cot", "[", 
       RowBox[{"\[Alpha]o", "+", 
        FractionBox[
         RowBox[{"\[Pi]", "-", "\[Beta]"}], "2"]}], "]"}]}]}], 
    RowBox[{"2", " ", 
     RowBox[{"(", 
      RowBox[{
       RowBox[{"Cot", "[", 
        RowBox[{"\[Alpha]o", "+", 
         FractionBox[
          RowBox[{"\[Pi]", "-", "\[Beta]"}], "2"]}], "]"}], "+", 
       RowBox[{"Tan", "[", 
        RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}], ")"}]}]]}], 
  ";"}]], "Input"]
}, Closed]],

Cell[CellGroupData[{

Cell["Lengths", "Subsection"],

Cell[CellGroupData[{

Cell["Laser Min", "Subsubsection"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"laserMin", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "S_", ",", "l_", ",", "\[Epsilon]_"}],
     "]"}], " ", ":=", " ", 
   SqrtBox[
    RowBox[{
     SuperscriptBox[
      RowBox[{"xmin", "[", 
       RowBox[{
       "\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
       "]"}], "2"], "+", 
     SuperscriptBox[
      RowBox[{
       RowBox[{"laser", "[", "\[Epsilon]", "]"}], "[", 
       RowBox[{"xmin", "[", 
        RowBox[{
        "\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
        "]"}], "]"}], "2"]}]]}], ";"}]], "Input"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{
  RowBox[{"laserMin", "[", 
   RowBox[{
   "\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
   "]"}], "//", "Simplify"}]], "Input"],

Cell[BoxData[
 RowBox[{
  FractionBox["1", "2"], " ", 
  SqrtBox[
   FractionBox[
    RowBox[{
     SuperscriptBox[
      RowBox[{"Sec", "[", "\[Epsilon]", "]"}], "2"], " ", 
     SuperscriptBox[
      RowBox[{"(", 
       RowBox[{"S", "+", 
        RowBox[{"2", " ", "l", " ", 
         RowBox[{"Tan", "[", 
          RowBox[{"\[Alpha]i", "+", 
           FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}], "2"]}], 
    SuperscriptBox[
     RowBox[{"(", 
      RowBox[{
       RowBox[{"Tan", "[", 
        RowBox[{"\[Alpha]i", "+", 
         FractionBox["\[Beta]", "2"]}], "]"}], "-", 
       RowBox[{"Tan", "[", 
        RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}], ")"}], 
     "2"]]]}]], "Output"]
}, Open  ]]
}, Closed]],

Cell[CellGroupData[{

Cell["Laser Min Alternative (Giulio)", "Subsubsection"],

Cell[BoxData[
 RowBox[{
  RowBox[{"laserMinAlt", "[", 
   RowBox[{
   "\[Beta]_", ",", "\[Alpha]i_", ",", "S_", ",", "l_", ",", "\[Epsilon]_"}], 
   "]"}], ":=", 
  RowBox[{
   RowBox[{"(", 
    RowBox[{
     RowBox[{"lExt", "[", 
      RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S"}], "]"}], "+", "l"}], 
    ")"}], "*", 
   RowBox[{"Sin", "[", 
    FractionBox[
     RowBox[{"\[Gamma]", "[", 
      RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "]"}], "*", 
   SuperscriptBox[
    RowBox[{"Sin", "[", 
     RowBox[{
      FractionBox[
       RowBox[{"\[Gamma]", "[", 
        RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "-", 
      RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], 
    RowBox[{"-", "1"}]]}]}]], "Input"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{
  RowBox[{"laserMinAlt", "[", 
   RowBox[{
   "\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
   "]"}], "//", "Simplify"}]], "Input"],

Cell[BoxData[
 RowBox[{
  FractionBox["1", "2"], " ", 
  RowBox[{"Csc", "[", 
   RowBox[{"\[Alpha]i", "+", 
    FractionBox["\[Beta]", "2"], "-", 
    RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], " ", 
  RowBox[{"(", 
   RowBox[{
    RowBox[{"S", " ", 
     RowBox[{"Cos", "[", 
      RowBox[{"\[Alpha]i", "+", 
       FractionBox["\[Beta]", "2"]}], "]"}]}], "+", 
    RowBox[{"2", " ", "l", " ", 
     RowBox[{"Sin", "[", 
      RowBox[{"\[Alpha]i", "+", 
       FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}]}]], "Output"]
}, Open  ]]
}, Closed]],

Cell[CellGroupData[{

Cell["Laser Max", "Subsubsection"],

Cell[BoxData[
 RowBox[{
  RowBox[{"laserMax", "[", 
   RowBox[{
   "\[Beta]_", ",", "\[Alpha]o_", ",", "S_", ",", "l_", ",", "\[Epsilon]_"}], 
   "]"}], ":=", 
  SqrtBox[
   RowBox[{
    SuperscriptBox[
     RowBox[{"xmax", "[", 
      RowBox[{
      "\[Beta]", ",", "\[Alpha]o", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
      "]"}], "2"], "+", 
    SuperscriptBox[
     RowBox[{
      RowBox[{"laser", "[", "\[Epsilon]", "]"}], "[", 
      RowBox[{"xmax", "[", 
       RowBox[{
       "\[Beta]", ",", "\[Alpha]o", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
       "]"}], "]"}], "2"]}]]}]], "Input"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{
  RowBox[{"laserMax", "[", 
   RowBox[{
   "\[Beta]", ",", "\[Alpha]o", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
   "]"}], "//", "Simplify"}]], "Input"],

Cell[BoxData[
 RowBox[{
  FractionBox["1", "2"], " ", 
  SqrtBox[
   FractionBox[
    RowBox[{
     SuperscriptBox[
      RowBox[{"Sec", "[", "\[Epsilon]", "]"}], "2"], " ", 
     SuperscriptBox[
      RowBox[{"(", 
       RowBox[{"S", "-", 
        RowBox[{"2", " ", "l", " ", 
         RowBox[{"Tan", "[", 
          RowBox[{"\[Alpha]o", "-", 
           FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}], "2"]}], 
    SuperscriptBox[
     RowBox[{"(", 
      RowBox[{
       RowBox[{"Tan", "[", 
        RowBox[{
         FractionBox["1", "2"], " ", 
         RowBox[{"(", 
          RowBox[{
           RowBox[{
            RowBox[{"-", "2"}], " ", "\[Alpha]o"}], "+", "\[Beta]"}], ")"}]}],
         "]"}], "+", 
       RowBox[{"Tan", "[", 
        RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}], ")"}], 
     "2"]]]}]], "Output"]
}, Open  ]]
}, Closed]],

Cell[CellGroupData[{

Cell["Laser Max Alternative (Giulio)", "Subsubsection"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"\[CapitalDelta]laser", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_", 
     ",", "\[Epsilon]_"}], "]"}], ":=", 
   RowBox[{
    RowBox[{"(", 
     RowBox[{
      RowBox[{"ir\[CapitalDelta]", "[", 
       RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S"}], 
       "]"}], "-", 
      RowBox[{
       RowBox[{"(", 
        RowBox[{
         RowBox[{"lExt", "[", 
          RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S"}], "]"}], "+", "l"}], 
        ")"}], "*", 
       SuperscriptBox[
        RowBox[{"Sin", "[", 
         RowBox[{
          FractionBox[
           RowBox[{"\[Gamma]", "[", 
            RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "-", 
          RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], 
        RowBox[{"-", "1"}]], "*", 
       RowBox[{"Sin", "[", 
        RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}]}], ")"}], "*", 
    RowBox[{"Sin", "[", 
     RowBox[{
      FractionBox[
       RowBox[{"\[Gamma]", "[", 
        RowBox[{"\[Beta]", ",", "\[Alpha]i"}], "]"}], "2"], "-", 
      RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], 
    SuperscriptBox[
     RowBox[{"Sin", "[", 
      RowBox[{
       FractionBox["\[Beta]", "2"], "-", "\[Alpha]o", "-", 
       RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], 
     RowBox[{"-", "1"}]]}]}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"laserMaxAlt", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_", 
     ",", "\[Epsilon]_"}], "]"}], ":=", 
   RowBox[{
    RowBox[{"laserMinAlt", "[", 
     RowBox[{
     "\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
     "]"}], "+", 
    RowBox[{"\[CapitalDelta]laser", "[", 
     RowBox[{
     "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l", ",", 
      "\[Epsilon]"}], "]"}]}]}], ";"}]], "Input"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{
  RowBox[{"\[CapitalDelta]laser", "[", 
   RowBox[{
   "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l", ",", 
    "\[Epsilon]"}], "]"}], "//", "Simplify"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{"-", 
   FractionBox["1", "2"]}], " ", 
  RowBox[{"Csc", "[", 
   RowBox[{"\[Alpha]o", "-", 
    FractionBox["\[Beta]", "2"], "+", 
    RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], " ", 
  RowBox[{"(", 
   RowBox[{
    RowBox[{"S", " ", 
     RowBox[{"Csc", "[", 
      RowBox[{"\[Alpha]i", "+", 
       FractionBox["\[Beta]", "2"]}], "]"}], " ", 
     RowBox[{"Csc", "[", 
      RowBox[{"\[Alpha]i", "-", "\[Alpha]o", "+", "\[Beta]"}], "]"}], " ", 
     RowBox[{"Sin", "[", 
      RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], "]"}], " ", 
     RowBox[{"Sin", "[", 
      RowBox[{"\[Alpha]i", "+", 
       FractionBox["\[Beta]", "2"], "-", 
       RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}]}], "-", 
    RowBox[{
     RowBox[{"(", 
      RowBox[{
       RowBox[{"2", " ", "l"}], "+", 
       RowBox[{"S", " ", 
        RowBox[{"Cot", "[", 
         RowBox[{"\[Alpha]i", "+", 
          FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}], " ", 
     RowBox[{"Sin", "[", 
      RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}]}], ")"}]}]], "Output"]
}, Open  ]],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{
  RowBox[{"laserMaxAlt", "[", 
   RowBox[{
   "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l", ",", 
    "\[Epsilon]"}], "]"}], "//", "Simplify"}]], "Input"],

Cell[BoxData[
 RowBox[{
  FractionBox["1", "2"], " ", 
  RowBox[{"(", 
   RowBox[{
    RowBox[{
     RowBox[{"Csc", "[", 
      RowBox[{"\[Alpha]i", "+", 
       FractionBox["\[Beta]", "2"], "-", 
       RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], " ", 
     RowBox[{"(", 
      RowBox[{
       RowBox[{"S", " ", 
        RowBox[{"Cos", "[", 
         RowBox[{"\[Alpha]i", "+", 
          FractionBox["\[Beta]", "2"]}], "]"}]}], "+", 
       RowBox[{"2", " ", "l", " ", 
        RowBox[{"Sin", "[", 
         RowBox[{"\[Alpha]i", "+", 
          FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}]}], "-", 
    RowBox[{
     RowBox[{"Csc", "[", 
      RowBox[{"\[Alpha]o", "-", 
       FractionBox["\[Beta]", "2"], "+", 
       RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}], " ", 
     RowBox[{"(", 
      RowBox[{
       RowBox[{"S", " ", 
        RowBox[{"Csc", "[", 
         RowBox[{"\[Alpha]i", "+", 
          FractionBox["\[Beta]", "2"]}], "]"}], " ", 
        RowBox[{"Csc", "[", 
         RowBox[{"\[Alpha]i", "-", "\[Alpha]o", "+", "\[Beta]"}], "]"}], " ", 
        
        RowBox[{"Sin", "[", 
         RowBox[{"\[Alpha]i", "+", "\[Alpha]o"}], "]"}], " ", 
        RowBox[{"Sin", "[", 
         RowBox[{"\[Alpha]i", "+", 
          FractionBox["\[Beta]", "2"], "-", 
          RowBox[{"Abs", "[", "\[Epsilon]", "]"}]}], "]"}]}], "-", 
       RowBox[{
        RowBox[{"(", 
         RowBox[{
          RowBox[{"2", " ", "l"}], "+", 
          RowBox[{"S", " ", 
           RowBox[{"Cot", "[", 
            RowBox[{"\[Alpha]i", "+", 
             FractionBox["\[Beta]", "2"]}], "]"}]}]}], ")"}], " ", 
        RowBox[{"Sin", "[", 
         RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "]"}]}]}], ")"}]}]}], 
   ")"}]}]], "Output"]
}, Open  ]]
}, Closed]]
}, Closed]]
}, Closed]],

Cell[CellGroupData[{

Cell["Graphical representation", "Section"],

Cell[CellGroupData[{

Cell["Graphical elements", "Subsection"],

Cell[BoxData[
 RowBox[{
  RowBox[{"laserColor", "=", "Green"}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{"diodeEmitterColor", "=", "Blue"}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{"diodeReceiverColor", "=", "Blue"}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"laserRay", "[", 
    RowBox[{"\[Epsilon]_", ",", "color_"}], "]"}], ":=", 
   RowBox[{"Plot", "[", 
    RowBox[{
     RowBox[{
      RowBox[{"laser", "[", "\[Epsilon]", "]"}], "[", "x", "]"}], ",", 
     RowBox[{"{", 
      RowBox[{"x", ",", "0", ",", "0.24"}], "}"}], ",", "\[IndentingNewLine]", 
     RowBox[{"PlotRange", "\[Rule]", 
      RowBox[{"{", 
       RowBox[{
        RowBox[{"{", 
         RowBox[{"0.15", ",", "0.27"}], "}"}], ",", 
        RowBox[{"{", 
         RowBox[{
          RowBox[{"-", "0.05"}], ",", "0.05"}], "}"}]}], "}"}]}], ",", 
     RowBox[{"AspectRatio", "\[Rule]", "Automatic"}], ",", 
     RowBox[{"ImageSize", "\[Rule]", "Large"}], ",", "\[IndentingNewLine]", 
     RowBox[{"PlotStyle", "\[Rule]", 
      RowBox[{"Directive", "[", "color", "]"}]}]}], "]"}]}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"intersectRay", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_", 
     ",", "\[Epsilon]_"}], "]"}], ":=", 
   RowBox[{"Plot", "[", 
    RowBox[{
     RowBox[{
      RowBox[{"laser", "[", "\[Epsilon]", "]"}], "[", "x", "]"}], ",", 
     RowBox[{"{", 
      RowBox[{"x", ",", 
       RowBox[{"xmin", "[", 
        RowBox[{
        "\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
        "]"}], ",", 
       RowBox[{"xmax", "[", 
        RowBox[{
        "\[Beta]", ",", "\[Alpha]o", ",", "S", ",", "l", ",", "\[Epsilon]"}], 
        "]"}]}], "}"}], ",", "\[IndentingNewLine]", 
     RowBox[{"PlotStyle", "\[Rule]", 
      RowBox[{"Directive", "[", 
       RowBox[{"Red", ",", "Dashed"}], "]"}]}]}], "]"}]}], ";"}]], "Input"],

Cell[BoxData[
 RowBox[{
  RowBox[{
   RowBox[{"diode", "[", 
    RowBox[{
    "\[Beta]_", ",", "\[Alpha]i_", ",", "\[Alpha]o_", ",", "S_", ",", "l_", 
     ",", "sign_", ",", "color_"}], "]"}], ":=", 
   RowBox[{"Plot", "[", "\[IndentingNewLine]", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{
       RowBox[{"sign", "*", 
        RowBox[{
         RowBox[{"diodeOuter", "[", 
          RowBox[{
          "\[Beta]", ",", "\[Alpha]i", ",", "\[Alpha]o", ",", "S", ",", "l"}],
           "]"}], "[", "x", "]"}]}], ",", 
       RowBox[{"sign", "*", 
        RowBox[{
         RowBox[{"diodeCenter", "[", 
          RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l"}], "]"}], 
         "[", "x", "]"}]}], ",", 
       RowBox[{"sign", "*", 
        RowBox[{
         RowBox[{"diodeInner", "[", 
          RowBox[{"\[Beta]", ",", "\[Alpha]i", ",", "S", ",", "l"}], "]"}], 
         "[", "x", "]"}]}]}], "}"}], ",", "\[IndentingNewLine]", 
     RowBox[{"{", 
      RowBox[{"x", ",", "l", ",", 
       RowBox[{"l", "+", "0.1"}]}], "}"}], ",", "\[IndentingNewLine]", 
     RowBox[{"Filling", "\[Rule]", 
      RowBox[{"{", 
       RowBox[{"1", "\[Rule]", 
        RowBox[{"{", "3", "}"}]}], "}"}]}], ",", 
     RowBox[{"FillingStyle", "\[Rule]", 
      RowBox[{"Directive", "[", 
       RowBox[{"color", ",", 
        RowBox[{"Opacity", "[", "0.1", "]"}]}], "]"}]}], ",", 
     "\[IndentingNewLine]", 
     RowBox[{"PlotStyle", "\[Rule]", 
      RowBox[{"{", 
       RowBox[{
        RowBox[{"Directive", "[", 
         RowBox[{"color", ",", 
          RowBox[{"Opacity", "[", "0.5", "]"}]}], "]"}], ",", 
        RowBox[{"Directive", "[", 
         RowBox[{"color", ",", "Dashed", ",", 
          RowBox[{"Opacity", "[", "0.5", "]"}], ",", "Thin"}], "]"}], ",", 
        RowBox[{"Directive", "[", 
         RowBox[{"color", ",", 
          RowBox[{"Opacity", "[", "0.5", "]"}]}], "]"}]}], "}"}]}]}], "]"}]}],
   ";"}]], "Input"],

Cell[BoxData[""], "Input"]
}, Closed]],

Cell[CellGroupData[{

Cell["Graphics", "Subsection"],

Cell[CellGroupData[{

Cell[BoxData[
 RowBox[{"Manipulate", "[", 
  RowBox[{
   RowBox[{"Show", "[", "\[IndentingNewLine]", 
    RowBox[{
     RowBox[{"laserRay", "[", 
      RowBox[{
       RowBox[{"\[Epsilon]", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", "laserColor"}], "]"}], ",", 
     "\[IndentingNewLine]", 
     RowBox[{"diode", "[", 
      RowBox[{
       RowBox[{"\[Beta]", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", 
       RowBox[{"\[Alpha]i", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", 
       RowBox[{"\[Alpha]o", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", "1", ",", 
       "diodeReceiverColor"}], "]"}], ",", "\[IndentingNewLine]", 
     RowBox[{"diode", "[", 
      RowBox[{
       RowBox[{"\[Beta]", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", 
       RowBox[{"\[Alpha]i", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", 
       RowBox[{"\[Alpha]o", "*", 
        RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
       RowBox[{"-", "1"}], ",", "diodeEmitterColor"}], "]"}], ",", 
     "\[IndentingNewLine]", 
     RowBox[{"If", "[", 
      RowBox[{
       RowBox[{
        RowBox[{
         RowBox[{"Abs", "[", "\[Epsilon]", "]"}], "*", 
         RowBox[{"\[Pi]", "/", "180"}]}], "<=", 
        RowBox[{"\[Epsilon]max", "[", 
         RowBox[{
          RowBox[{"\[Beta]", "*", 
           RowBox[{"\[Pi]", "/", "180"}]}], ",", 
          RowBox[{"\[Alpha]i", "*", 
           RowBox[{"\[Pi]", "/", "180"}]}], ",", 
          RowBox[{"\[Alpha]o", "*", 
           RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l"}], "]"}]}], 
       ",", "\[IndentingNewLine]", 
       RowBox[{"{", 
        RowBox[{
         RowBox[{"intersectRay", "[", 
          RowBox[{
           RowBox[{"\[Beta]", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}], ",", 
           RowBox[{"\[Alpha]i", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}], ",", 
           RowBox[{"\[Alpha]o", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
           RowBox[{"\[Epsilon]", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}]}], "]"}], ",", 
         RowBox[{"Graphics", "[", 
          RowBox[{"Text", "[", 
           RowBox[{
            RowBox[{"\"\<Laser min:\>\"", " ", "<>", " ", 
             RowBox[{"ToString", "[", 
              RowBox[{"laserMin", "[", 
               RowBox[{
                RowBox[{"\[Beta]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]i", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
                RowBox[{"\[Epsilon]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}]}], "]"}], "]"}]}], ",", 
            RowBox[{"{", 
             RowBox[{"0.18", ",", "0.04"}], "}"}]}], "]"}], "]"}], ",", 
         "\[IndentingNewLine]", 
         RowBox[{"Graphics", "[", 
          RowBox[{"Text", "[", 
           RowBox[{
            RowBox[{"\"\<Laser min (alt):\>\"", " ", "<>", " ", 
             RowBox[{"ToString", "[", 
              RowBox[{"laserMinAlt", "[", 
               RowBox[{
                RowBox[{"\[Beta]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]i", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
                RowBox[{"\[Epsilon]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}]}], "]"}], "]"}]}], ",", 
            RowBox[{"{", 
             RowBox[{"0.18", ",", "0.037"}], "}"}]}], "]"}], "]"}], ",", 
         "\[IndentingNewLine]", 
         RowBox[{"Graphics", "[", 
          RowBox[{"Text", "[", 
           RowBox[{
            RowBox[{"\"\<Laser max:\>\"", " ", "<>", " ", 
             RowBox[{"ToString", "[", 
              RowBox[{"laserMax", "[", 
               RowBox[{
                RowBox[{"\[Beta]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]o", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
                RowBox[{"\[Epsilon]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}]}], "]"}], "]"}]}], ",", 
            RowBox[{"{", 
             RowBox[{"0.18", ",", "0.034"}], "}"}]}], "]"}], "]"}], ",", 
         "\[IndentingNewLine]", 
         RowBox[{"Graphics", "[", 
          RowBox[{"Text", "[", 
           RowBox[{
            RowBox[{"\"\<Laser max (alt):\>\"", " ", "<>", " ", 
             RowBox[{"ToString", "[", 
              RowBox[{"laserMaxAlt", "[", 
               RowBox[{
                RowBox[{"\[Beta]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]i", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]o", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
                RowBox[{"\[Epsilon]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}]}], "]"}], "]"}]}], ",", 
            RowBox[{"{", 
             RowBox[{"0.18", ",", "0.031"}], "}"}]}], "]"}], "]"}], ",", 
         "\[IndentingNewLine]", 
         RowBox[{"Graphics", "[", 
          RowBox[{"Text", "[", 
           RowBox[{
            RowBox[{"\"\<\[CapitalDelta]laser (alt):\>\"", " ", "<>", " ", 
             RowBox[{"ToString", "[", 
              RowBox[{"\[CapitalDelta]laser", "[", 
               RowBox[{
                RowBox[{"\[Beta]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]i", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", 
                RowBox[{"\[Alpha]o", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l", ",", 
                RowBox[{"\[Epsilon]", "*", 
                 RowBox[{"\[Pi]", "/", "180"}]}]}], "]"}], "]"}]}], ",", 
            RowBox[{"{", 
             RowBox[{"0.18", ",", "0.028"}], "}"}]}], "]"}], "]"}]}], "}"}], 
       ",", "\[IndentingNewLine]", 
       RowBox[{"Plot", "[", 
        RowBox[{
         RowBox[{"{", "}"}], ",", 
         RowBox[{"{", 
          RowBox[{"x", ",", "0", ",", "0.1"}], "}"}]}], "]"}]}], "]"}]}], 
    "\[IndentingNewLine]", "]"}], ",", "\[IndentingNewLine]", 
   RowBox[{"{", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{"\[Epsilon]", ",", 
       RowBox[{
        RowBox[{"-", 
         RowBox[{"\[Epsilon]max", "[", 
          RowBox[{
           RowBox[{"\[Beta]", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}], ",", 
           RowBox[{"\[Alpha]i", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}], ",", 
           RowBox[{"\[Alpha]o", "*", 
            RowBox[{"\[Pi]", "/", "180"}]}], ",", "S", ",", "l"}], "]"}]}], 
        "*", 
        RowBox[{"180", "/", "\[Pi]"}]}], ",", 
       "\"\<Ray angle \[Epsilon]\>\""}], "}"}], ",", 
     RowBox[{"-", "90"}], ",", "90", ",", "0.2", ",", 
     RowBox[{"Appearance", "\[Rule]", "\"\<Open\>\""}]}], "}"}], ",", 
   "\[IndentingNewLine]", 
   RowBox[{"{", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{"\[Beta]", ",", "50", ",", "\"\<Angle between \[Beta]\>\""}], 
      "}"}], ",", "40", ",", "60", ",", 
     RowBox[{"Appearance", "\[Rule]", "\"\<Open\>\""}]}], "}"}], ",", 
   "\[IndentingNewLine]", 
   RowBox[{"{", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{
      "\[Alpha]i", ",", "25", ",", 
       "\"\<Inner off center angle \[Alpha]i\>\""}], "}"}], ",", "20", ",", 
     "30", ",", 
     RowBox[{"Appearance", "\[Rule]", "\"\<Open\>\""}]}], "}"}], ",", 
   "\[IndentingNewLine]", 
   RowBox[{"{", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{
      "\[Alpha]o", ",", "10", ",", 
       "\"\<Outer off center angle \[Alpha]o\>\""}], "}"}], ",", "5", ",", 
     "15", ",", 
     RowBox[{"Appearance", "\[Rule]", "\"\<Open\>\""}]}], "}"}], ",", 
   "\[IndentingNewLine]", 
   RowBox[{"{", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{"S", ",", "0.034", ",", "\"\<Sensor housing length\>\""}], 
      "}"}], ",", "0.01", ",", "0.05", ",", 
     RowBox[{"Appearance", "\[Rule]", "\"\<Open\>\""}]}], "}"}], ",", 
   "\[IndentingNewLine]", 
   RowBox[{"{", 
    RowBox[{
     RowBox[{"{", 
      RowBox[{"l", ",", "0.1696", ",", "\"\<Roomba 'fake' radius l\>\""}], 
      "}"}], ",", "0.1", ",", "0.3", ",", 
     RowBox[{"Appearance", "\[Rule]", "\"\<Open\>\""}]}], "}"}], ",", 
   "\[IndentingNewLine]", 
   RowBox[{"ControlPlacement", "\[Rule]", "Right"}]}], "\[IndentingNewLine]", 
  "]"}]], "Input"],

Cell[BoxData[
 TagBox[
  StyleBox[
   DynamicModuleBox[{$CellContext`l$$ = 0.1696, $CellContext`S$$ = 
    0.034, $CellContext`\[Alpha]i$$ = 25, $CellContext`\[Alpha]o$$ = 
    10, $CellContext`\[Beta]$$ = 50, $CellContext`\[Epsilon]$$ = 
    1.6075636893277438`, Typeset`show$$ = True, Typeset`bookmarkList$$ = {}, 
    Typeset`bookmarkMode$$ = "Menu", Typeset`animator$$, Typeset`animvar$$ = 
    1, Typeset`name$$ = "\"untitled\"", Typeset`specs$$ = {{{
       Hold[$CellContext`\[Epsilon]$$], ((-180)/Pi) 
       ArcSin[(((Rational[1, 2] $CellContext`S$$) 
           Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
               Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
               Rational[1, 180] Pi) $CellContext`\[Beta]$$]) 
          Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
              Rational[1, 180] 
              Pi) $CellContext`\[Alpha]o$$]) ((((
               Rational[1, 4] $CellContext`S$$^2) 
              Csc[Rational[
                  1, 2] ((Rational[1, 90] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$)]^2) 
             Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                  Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                  Rational[1, 180] Pi) $CellContext`\[Beta]$$]^2) 
            Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                 Rational[1, 180] 
                 Pi) $CellContext`\[Alpha]o$$]^2 + ((($CellContext`S$$ 
               Cot[Rational[
                  1, 2] ((Rational[1, 90] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$)]) 
              Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                  Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                  Rational[1, 180] Pi) $CellContext`\[Beta]$$]) 
             Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                 Rational[1, 180] 
                 Pi) $CellContext`\[Alpha]o$$]) ($CellContext`l$$ + (
               Rational[-1, 2] $CellContext`S$$) 
              Tan[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + 
                Rational[
                  1, 2] (-
                   Pi + (Rational[1, 180] 
                    Pi) $CellContext`\[Beta]$$)]) + ($CellContext`l$$ + (
               Rational[-1, 2] $CellContext`S$$) 
              Tan[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + 
                Rational[
                  1, 2] (-
                   Pi + (Rational[1, 180] Pi) $CellContext`\[Beta]$$)])^2)^
          Rational[-1, 2]], "Ray angle \[Epsilon]"}, -90, 90, 0.2}, {{
       Hold[$CellContext`\[Beta]$$], 50, "Angle between \[Beta]"}, 40, 60}, {{
      
       Hold[$CellContext`\[Alpha]i$$], 25, 
       "Inner off center angle \[Alpha]i"}, 20, 30}, {{
       Hold[$CellContext`\[Alpha]o$$], 10, 
       "Outer off center angle \[Alpha]o"}, 5, 15}, {{
       Hold[$CellContext`S$$], 0.034, "Sensor housing length"}, 0.01, 0.05}, {{
       Hold[$CellContext`l$$], 0.1696, "Roomba 'fake' radius l"}, 0.1, 0.3}}, 
    Typeset`size$$ = {576., {238., 242.}}, Typeset`update$$ = 0, 
    Typeset`initDone$$, Typeset`skipInitDone$$ = 
    True, $CellContext`\[Epsilon]$483$$ = 0, $CellContext`\[Beta]$488$$ = 
    0, $CellContext`\[Alpha]i$489$$ = 0, $CellContext`\[Alpha]o$490$$ = 
    0, $CellContext`S$491$$ = 0, $CellContext`l$492$$ = 0}, 
    DynamicBox[Manipulate`ManipulateBoxes[
     1, StandardForm, 
      "Variables" :> {$CellContext`l$$ = 0.1696, $CellContext`S$$ = 
        0.034, $CellContext`\[Alpha]i$$ = 25, $CellContext`\[Alpha]o$$ = 
        10, $CellContext`\[Beta]$$ = 
        50, $CellContext`\[Epsilon]$$ = ((-180)/Pi) 
         ArcSin[(((Rational[1, 2] $CellContext`S$$) 
             Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                 Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                 Rational[1, 180] Pi) $CellContext`\[Beta]$$]) 
            Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                Rational[1, 180] 
                Pi) $CellContext`\[Alpha]o$$]) ((((
                 Rational[1, 4] $CellContext`S$$^2) 
                Csc[Rational[
                    1, 2] ((Rational[1, 90] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$)]^2) 
               Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$]^2) 
              Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                   Rational[1, 180] 
                   Pi) $CellContext`\[Alpha]o$$]^2 + ((($CellContext`S$$ 
                 Cot[Rational[
                    1, 
                    2] ((Rational[1, 90] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$)]) 
                Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$]) 
               Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                   Rational[1, 180] 
                   Pi) $CellContext`\[Alpha]o$$]) ($CellContext`l$$ + (
                 Rational[-1, 2] $CellContext`S$$) 
                Tan[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + 
                  Rational[
                    1, 2] (-
                    Pi + (Rational[1, 180] 
                    Pi) $CellContext`\[Beta]$$)]) + ($CellContext`l$$ + (
                 Rational[-1, 2] $CellContext`S$$) 
                Tan[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + 
                  Rational[
                    1, 2] (-
                    Pi + (Rational[1, 180] Pi) $CellContext`\[Beta]$$)])^2)^
            Rational[-1, 2]]}, "ControllerVariables" :> {
        Hold[$CellContext`\[Epsilon]$$, $CellContext`\[Epsilon]$483$$, 0], 
        Hold[$CellContext`\[Beta]$$, $CellContext`\[Beta]$488$$, 0], 
        Hold[$CellContext`\[Alpha]i$$, $CellContext`\[Alpha]i$489$$, 0], 
        Hold[$CellContext`\[Alpha]o$$, $CellContext`\[Alpha]o$490$$, 0], 
        Hold[$CellContext`S$$, $CellContext`S$491$$, 0], 
        Hold[$CellContext`l$$, $CellContext`l$492$$, 0]}, 
      "OtherVariables" :> {
       Typeset`show$$, Typeset`bookmarkList$$, Typeset`bookmarkMode$$, 
        Typeset`animator$$, Typeset`animvar$$, Typeset`name$$, 
        Typeset`specs$$, Typeset`size$$, Typeset`update$$, Typeset`initDone$$,
         Typeset`skipInitDone$$}, "Body" :> Show[
        $CellContext`laserRay[$CellContext`\[Epsilon]$$ (Pi/
          180), $CellContext`laserColor], 
        $CellContext`diode[$CellContext`\[Beta]$$ (Pi/
          180), $CellContext`\[Alpha]i$$ (Pi/180), $CellContext`\[Alpha]o$$ (
          Pi/180), $CellContext`S$$, $CellContext`l$$, 
         1, $CellContext`diodeReceiverColor], 
        $CellContext`diode[$CellContext`\[Beta]$$ (Pi/
          180), $CellContext`\[Alpha]i$$ (Pi/180), $CellContext`\[Alpha]o$$ (
          Pi/
          180), $CellContext`S$$, $CellContext`l$$, -1, \
$CellContext`diodeEmitterColor], 
        If[
        Abs[$CellContext`\[Epsilon]$$] (Pi/
           180) <= $CellContext`\[Epsilon]max[$CellContext`\[Beta]$$ (Pi/
            180), $CellContext`\[Alpha]i$$ (Pi/
            180), $CellContext`\[Alpha]o$$ (Pi/
            180), $CellContext`S$$, $CellContext`l$$], {
          $CellContext`intersectRay[$CellContext`\[Beta]$$ (Pi/
            180), $CellContext`\[Alpha]i$$ (Pi/
            180), $CellContext`\[Alpha]o$$ (Pi/
            180), $CellContext`S$$, $CellContext`l$$, \
$CellContext`\[Epsilon]$$ (Pi/180)], 
          Graphics[
           Text[
            StringJoin["Laser min:", 
             ToString[
              $CellContext`laserMin[$CellContext`\[Beta]$$ (Pi/
                180), $CellContext`\[Alpha]i$$ (Pi/
                180), $CellContext`S$$, $CellContext`l$$, $CellContext`\
\[Epsilon]$$ (Pi/180)]]], {0.18, 0.04}]], 
          Graphics[
           Text[
            StringJoin["Laser min (alt):", 
             ToString[
              $CellContext`laserMinAlt[$CellContext`\[Beta]$$ (Pi/
                180), $CellContext`\[Alpha]i$$ (Pi/
                180), $CellContext`S$$, $CellContext`l$$, $CellContext`\
\[Epsilon]$$ (Pi/180)]]], {0.18, 0.037}]], 
          Graphics[
           Text[
            StringJoin["Laser max:", 
             ToString[
              $CellContext`laserMax[$CellContext`\[Beta]$$ (Pi/
                180), $CellContext`\[Alpha]o$$ (Pi/
                180), $CellContext`S$$, $CellContext`l$$, $CellContext`\
\[Epsilon]$$ (Pi/180)]]], {0.18, 0.034}]], 
          Graphics[
           Text[
            StringJoin["Laser max (alt):", 
             ToString[
              $CellContext`laserMaxAlt[$CellContext`\[Beta]$$ (Pi/
                180), $CellContext`\[Alpha]i$$ (Pi/
                180), $CellContext`\[Alpha]o$$ (Pi/
                180), $CellContext`S$$, $CellContext`l$$, $CellContext`\
\[Epsilon]$$ (Pi/180)]]], {0.18, 0.031}]], 
          Graphics[
           Text[
            StringJoin["\[CapitalDelta]laser (alt):", 
             ToString[
              $CellContext`\[CapitalDelta]laser[$CellContext`\[Beta]$$ (Pi/
                180), $CellContext`\[Alpha]i$$ (Pi/
                180), $CellContext`\[Alpha]o$$ (Pi/
                180), $CellContext`S$$, $CellContext`l$$, $CellContext`\
\[Epsilon]$$ (Pi/180)]]], {0.18, 0.028}]]}, 
         Plot[{}, {$CellContext`x, 0, 0.1}]]], 
      "Specifications" :> {{{$CellContext`\[Epsilon]$$, ((-180)/Pi) 
          ArcSin[(((Rational[1, 2] $CellContext`S$$) 
              Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                  Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                  Rational[1, 180] Pi) $CellContext`\[Beta]$$]) 
             Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                 Rational[1, 180] 
                 Pi) $CellContext`\[Alpha]o$$]) ((((
                  Rational[1, 4] $CellContext`S$$^2) 
                 Csc[Rational[
                    1, 2] ((Rational[1, 90] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$)]^2) 
                Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$]^2) 
               Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] 
                    Pi) $CellContext`\[Alpha]o$$]^2 + ((($CellContext`S$$ 
                  Cot[Rational[
                    1, 2] ((Rational[1, 90] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$)]) 
                 Csc[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[-1, 180] Pi) $CellContext`\[Alpha]o$$ + (
                    Rational[1, 180] Pi) $CellContext`\[Beta]$$]) 
                Sin[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + (
                    Rational[1, 180] 
                    Pi) $CellContext`\[Alpha]o$$]) ($CellContext`l$$ + (
                  Rational[-1, 2] $CellContext`S$$) 
                 Tan[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + 
                   Rational[
                    1, 2] (-
                    Pi + (Rational[1, 180] 
                    Pi) $CellContext`\[Beta]$$)]) + ($CellContext`l$$ + (
                  Rational[-1, 2] $CellContext`S$$) 
                 Tan[(Rational[1, 180] Pi) $CellContext`\[Alpha]i$$ + 
                   Rational[
                    1, 2] (-
                    Pi + (Rational[1, 180] Pi) $CellContext`\[Beta]$$)])^2)^
             Rational[-1, 2]], "Ray angle \[Epsilon]"}, -90, 90, 0.2, 
         Appearance -> 
         "Open"}, {{$CellContext`\[Beta]$$, 50, "Angle between \[Beta]"}, 40, 
         60, Appearance -> 
         "Open"}, {{$CellContext`\[Alpha]i$$, 25, 
          "Inner off center angle \[Alpha]i"}, 20, 30, Appearance -> 
         "Open"}, {{$CellContext`\[Alpha]o$$, 10, 
          "Outer off center angle \[Alpha]o"}, 5, 15, Appearance -> 
         "Open"}, {{$CellContext`S$$, 0.034, "Sensor housing length"}, 0.01, 
         0.05, Appearance -> 
         "Open"}, {{$CellContext`l$$, 0.1696, "Roomba 'fake' radius l"}, 0.1, 
         0.3, Appearance -> "Open"}}, 
      "Options" :> {ControlPlacement -> Right}, "DefaultOptions" :> {}],
     ImageSizeCache->{977., {267., 274.}},
     SingleEvaluation->True],
    Deinitialization:>None,
    DynamicModuleValues:>{},
    SynchronousInitialization->True,
    UnsavedVariables:>{Typeset`initDone$$},
    UntrackedVariables:>{Typeset`size$$}], "Manipulate",
   Deployed->True,
   StripOnInput->False],
  Manipulate`InterpretManipulate[1]]], "Output"]
}, Open  ]]
}, Open  ]]
}, Open  ]]
}, Open  ]]
},
WindowSize->{1364, 747},
Visible->True,
ScrollingOptions->{"VerticalScrollRange"->Fit},
ShowCellBracket->Automatic,
CellContext->Notebook,
TrackCellChangeTimes->False,
FrontEndVersion->"10.0 for Linux x86 (64-bit) (June 27, 2014)",
StyleDefinitions->"Default.nb"
]
(* End of Notebook Content *)

(* Internal cache information *)
(*CellTagsOutline
CellTagsIndex->{}
*)
(*CellTagsIndex
CellTagsIndex->{}
*)
(*NotebookFileOutline
Notebook[{
Cell[CellGroupData[{
Cell[1486, 35, 39, 0, 93, "Title"],
Cell[CellGroupData[{
Cell[1550, 39, 37, 0, 65, "Section"],
Cell[1590, 41, 181, 5, 32, "Input"],
Cell[1774, 48, 312, 10, 63, "Input"],
Cell[2089, 60, 742, 22, 64, "Input"],
Cell[2834, 84, 1051, 31, 81, "Input"],
Cell[3888, 117, 658, 19, 81, "Input"],
Cell[CellGroupData[{
Cell[4571, 140, 180, 5, 32, "Input"],
Cell[4754, 147, 1595, 46, 83, "Output"]
}, Open  ]]
}, Open  ]],
Cell[CellGroupData[{
Cell[6398, 199, 40, 0, 65, "Section"],
Cell[CellGroupData[{
Cell[6463, 203, 36, 0, 37, "Subsection"],
Cell[6502, 205, 207, 6, 26, "Input"],
Cell[6712, 213, 717, 23, 32, "Input"],
Cell[7432, 238, 351, 10, 32, "Input"],
Cell[7786, 250, 603, 20, 32, "Input"]
}, Closed]],
Cell[CellGroupData[{
Cell[8426, 275, 35, 0, 36, "Subsection"],
Cell[CellGroupData[{
Cell[8486, 279, 347, 10, 26, "Input"],
Cell[8836, 291, 820, 27, 67, "Output"]
}, Open  ]],
Cell[9671, 321, 915, 30, 93, "Input"],
Cell[CellGroupData[{
Cell[10611, 355, 349, 10, 32, "Input"],
Cell[10963, 367, 604, 19, 67, "Output"]
}, Open  ]],
Cell[11582, 389, 715, 23, 93, "Input"]
}, Closed]],
Cell[CellGroupData[{
Cell[12334, 417, 29, 0, 37, "Subsection"],
Cell[CellGroupData[{
Cell[12388, 421, 34, 0, 29, "Subsubsection"],
Cell[12425, 423, 645, 20, 41, "Input"],
Cell[CellGroupData[{
Cell[13095, 447, 177, 5, 32, "Input"],
Cell[13275, 454, 704, 23, 79, "Output"]
}, Open  ]]
}, Closed]],
Cell[CellGroupData[{
Cell[14028, 483, 55, 0, 29, "Subsubsection"],
Cell[14086, 485, 737, 23, 58, "Input"],
Cell[CellGroupData[{
Cell[14848, 512, 180, 5, 32, "Input"],
Cell[15031, 519, 530, 16, 48, "Output"]
}, Open  ]]
}, Closed]],
Cell[CellGroupData[{
Cell[15610, 541, 34, 0, 29, "Subsubsection"],
Cell[15647, 543, 599, 19, 41, "Input"],
Cell[CellGroupData[{
Cell[16271, 566, 177, 5, 32, "Input"],
Cell[16451, 573, 834, 28, 79, "Output"]
}, Open  ]]
}, Closed]],
Cell[CellGroupData[{
Cell[17334, 607, 55, 0, 29, "Subsubsection"],
Cell[17392, 609, 1407, 40, 117, "Input"],
Cell[18802, 651, 534, 15, 32, "Input"],
Cell[CellGroupData[{
Cell[19361, 670, 208, 5, 32, "Input"],
Cell[19572, 677, 1085, 31, 48, "Output"]
}, Open  ]],
Cell[CellGroupData[{
Cell[20694, 713, 199, 5, 32, "Input"],
Cell[20896, 720, 1741, 50, 86, "Output"]
}, Open  ]]
}, Closed]]
}, Closed]]
}, Closed]],
Cell[CellGroupData[{
Cell[22710, 778, 43, 0, 51, "Section"],
Cell[CellGroupData[{
Cell[22778, 782, 40, 0, 45, "Subsection"],
Cell[22821, 784, 80, 2, 26, "Input"],
Cell[22904, 788, 86, 2, 32, "Input"],
Cell[22993, 792, 87, 2, 32, "Input"],
Cell[23083, 796, 858, 22, 77, "Input"],
Cell[23944, 820, 837, 23, 55, "Input"],
Cell[24784, 845, 1928, 51, 121, "Input"],
Cell[26715, 898, 26, 0, 32, "Input"]
}, Closed]],
Cell[CellGroupData[{
Cell[26778, 903, 30, 0, 37, "Subsection"],
Cell[CellGroupData[{
Cell[26833, 907, 8407, 206, 473, "Input"],
Cell[35243, 1115, 12961, 242, 558, "Output"]
}, Open  ]]
}, Open  ]]
}, Open  ]]
}, Open  ]]
}
]
*)

(* End of internal cache information *)

(* NotebookSignature @up7rHOsnagp8Bgt7mBI1qBH *)

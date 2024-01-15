from pathlib import Path

from pybind11_stubgen import (
    BaseParser,
    ExtractSignaturesFromPybind11Docstrings,
    FilterClassMembers,
    FilterInvalidIdentifiers,
    FilterPybindInternals,
    FilterTypingModuleAttributes,
    FixBuiltinTypes,
    FixCurrentModulePrefixInTypeNames,
    FixMissing__all__Attribute,
    FixMissingEnumMembersAnnotation,
    FixMissingFixedSizeImport,
    FixMissingImports,
    FixMissingNoneHashFieldAnnotation,
    FixNumpyArrayDimTypeVar,
    FixNumpyArrayFlags,
    FixNumpyDtype,
    FixPEP585CollectionNames,
    FixPybind11EnumStrDoc,
    FixRedundantBuiltinsAnnotation,
    FixRedundantMethodsFromBuiltinObject,
    FixScipyTypeArguments,
    FixTypingTypeNames,
    FixValueReprRandomAddress,
    IParser,
    LogErrors,
    LoggerData,
    OverridePrintSafeValues,
    ParserDispatchMixin,
    Printer,
    RemoveSelfAnnotation,
    ReplaceReadWritePropertyWithField,
    RewritePybind11EnumValueRepr,
    SuggestCxxSignatureFix,
    Writer,
    run,
)


def stub_parser() -> IParser:
    error_handlers_top: list[type] = [
        LoggerData,
        # IgnoreAllErrors,  # args.ignore_all_errors
        # IgnoreInvalidIdentifierErrors, # args.ignore_invalid_identifiers
        # IgnoreInvalidExpressionErrors, # args.ignore_invalid_expressions
        # IgnoreUnresolvedNameErrors, # args.ignore_unresolved_names
    ]
    error_handlers_bottom: list[type] = [
        LogErrors,
        SuggestCxxSignatureFix,
        # TerminateOnFatalErrors, # args.exit_code
    ]

    numpy_fixes: list[type] = [
        FixNumpyArrayDimTypeVar,  # args.numpy_array_use_type_var
    ]

    class Parser(
        *error_handlers_top,  # type: ignore[misc]
        # FixMissing__future__AnnotationsImport,  # ruff: PYI044
        FixMissing__all__Attribute,
        FixMissingNoneHashFieldAnnotation,
        FixMissingImports,
        FilterTypingModuleAttributes,
        FixPEP585CollectionNames,
        FixTypingTypeNames,
        FixScipyTypeArguments,
        FixMissingFixedSizeImport,
        FixMissingEnumMembersAnnotation,
        OverridePrintSafeValues,
        *numpy_fixes,  # type: ignore[misc]
        FixNumpyDtype,
        FixNumpyArrayFlags,
        FixCurrentModulePrefixInTypeNames,
        FixBuiltinTypes,
        RewritePybind11EnumValueRepr,
        FilterClassMembers,
        ReplaceReadWritePropertyWithField,
        FilterInvalidIdentifiers,
        FixValueReprRandomAddress,
        FixRedundantBuiltinsAnnotation,
        FilterPybindInternals,
        FixRedundantMethodsFromBuiltinObject,
        RemoveSelfAnnotation,
        FixPybind11EnumStrDoc,
        ExtractSignaturesFromPybind11Docstrings,
        ParserDispatchMixin,
        BaseParser,
        *error_handlers_bottom,  # type: ignore[misc]
    ):
        pass

    parser = Parser()

    # if args.enum_class_locations:
    #     parser.set_pybind11_enum_locations(dict(args.enum_class_locations))
    # if args.ignore_invalid_identifiers is not None:
    #     parser.set_ignored_invalid_identifiers(args.ignore_invalid_identifiers)
    # if args.ignore_invalid_expressions is not None:
    #     parser.set_ignored_invalid_expressions(args.ignore_invalid_expressions)
    # if args.ignore_unresolved_names is not None:
    #     parser.set_ignored_unresolved_names(args.ignore_unresolved_names)
    # if args.print_safe_value_reprs is not None:
    #     parser.set_print_safe_value_pattern(args.print_safe_value_reprs)

    return parser


if __name__ == "__main__":
    run(
        parser=stub_parser(),
        printer=Printer(invalid_expr_as_ellipses=True),
        module_name="mplib",  # args.module_name
        out_dir=Path("stubs"),
        sub_dir=None,
        dry_run=False,
        writer=Writer(stub_ext="pyi"),
    )

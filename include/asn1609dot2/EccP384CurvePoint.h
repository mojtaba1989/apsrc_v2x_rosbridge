/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2BaseTypes"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_EccP384CurvePoint_H_
#define	_EccP384CurvePoint_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>
#include <NULL.h>
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum EccP384CurvePoint_PR {
	EccP384CurvePoint_PR_NOTHING,	/* No components present */
	EccP384CurvePoint_PR_x_only,
	EccP384CurvePoint_PR_fill,
	EccP384CurvePoint_PR_compressed_y_0,
	EccP384CurvePoint_PR_compressed_y_1,
	EccP384CurvePoint_PR_uncompressedP384
} EccP384CurvePoint_PR;

/* EccP384CurvePoint */
typedef struct EccP384CurvePoint {
	EccP384CurvePoint_PR present;
	union EccP384CurvePoint_u {
		OCTET_STRING_t	 x_only;
		NULL_t	 fill;
		OCTET_STRING_t	 compressed_y_0;
		OCTET_STRING_t	 compressed_y_1;
		struct EccP384CurvePoint__uncompressedP384 {
			OCTET_STRING_t	 x;
			OCTET_STRING_t	 y;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} uncompressedP384;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} EccP384CurvePoint_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_EccP384CurvePoint;
extern asn_CHOICE_specifics_t asn_SPC_EccP384CurvePoint_specs_1;
extern asn_TYPE_member_t asn_MBR_EccP384CurvePoint_1[5];
extern asn_per_constraints_t asn_PER_type_EccP384CurvePoint_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _EccP384CurvePoint_H_ */
#include <asn_internal.h>

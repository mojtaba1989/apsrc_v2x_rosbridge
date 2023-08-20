/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2BaseTypes"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_Psid_H_
#define	_Psid_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Psid */
typedef unsigned long	 Psid_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_Psid_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_Psid;
extern const asn_INTEGER_specifics_t asn_SPC_Psid_specs_1;
asn_struct_free_f Psid_free;
asn_struct_print_f Psid_print;
asn_constr_check_f Psid_constraint;
ber_type_decoder_f Psid_decode_ber;
der_type_encoder_f Psid_encode_der;
xer_type_decoder_f Psid_decode_xer;
xer_type_encoder_f Psid_encode_xer;
oer_type_decoder_f Psid_decode_oer;
oer_type_encoder_f Psid_encode_oer;
per_type_decoder_f Psid_decode_uper;
per_type_encoder_f Psid_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _Psid_H_ */
#include <asn_internal.h>
